// Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
// Copyright (c) 2022 Hans Baier (hansfbaier@gmail.com)
// SPDX-License-Identifier: Apache-2.0

#include "CH56x_common.h"
#include "CH56xusb30_LIB.h"
#include "CH56x_usb30.h"
#include "CH56x_usb20.h"
#include "CH56x_uart.h"

#include "debug.h"

/* Global define */
#define FREQ_SYS 96000000
//#define  FREQ_SYS     120000000
#define UART1_BAUD 115200

void DebugInit(UINT32 baudrate)
{
    UINT32 x;
    UINT32 t = FREQ_SYS;
    x = 10 * t * 2 / 16 / baudrate;
    x = (x + 5) / 10;
    R8_UART1_DIV = 1;
    R16_UART1_DL = x;
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R32_PA_SMT |= (1 << 8) | (1 << 7);
    R32_PA_DIR |= (1 << 8);
}

__attribute__((aligned(16))) UINT8 out_buf0[4096] __attribute__((section(".dmadata")));
__attribute__((aligned(16))) UINT8 in_buf0[4096] __attribute__((section(".dmadata")));
__attribute__((aligned(16))) UINT8 out_buf1[4096] __attribute__((section(".dmadata")));
__attribute__((aligned(16))) UINT8 in_buf1[4096] __attribute__((section(".dmadata")));

//////////////////////////////////////// HSPI ////////////////////////////////////
#define DMA_Len 4096

/* Shared variables */

volatile int HSPI_Rx_End_Err; // 0=No Error else >0 Error code

volatile int In_FIFO[2];
volatile int In_FIFO_Write_Pos = 0;
volatile int In_FIFO_Read_Pos  = 0;

/*TODO
volatile int Out_FIFO[2];
volatile int Out_FIFO_Write_Pos = 0;
volatile int Out_FIFO_Read_Pos  = 0;
*/

void HSPI_GPIO_Init(void)
{
    // TX GPIO PA9/PA11/PA21 Push-pull output
    R32_PA_DIR |= (1 << 9) | (1 << 11) | (1 << 21);

    // clk 16mA
    R32_PA_DRV |= (1 << 11);

    // RX GPIO PA10 Push-pull output
    R32_PA_DIR |= (1 << 10);
}

void HSPI_Init(void)
{
    // GPIO Cfg
    HSPI_GPIO_Init();

    // if we don't wait here we will get random CRC errors
    mDelaymS(100);

    // Clear HSPI config
    R8_HSPI_CFG &= ~(RB_HSPI_MODE | RB_HSPI_MSK_SIZE);

    // Enable Host mode
    R8_HSPI_CFG |= RB_HSPI_MODE;

    // Data size
    R8_HSPI_CFG |= RB_HSPI_DAT32_MOD;

    // ACK mode 0 (Hardware auto-answer mode is used in burst mode, not in normal mode)
    R8_HSPI_CFG &= ~RB_HSPI_HW_ACK;

    // enable DMA buffer toggling
    R8_HSPI_CFG |= RB_HSPI_RX_TOG_EN;
    R8_HSPI_CFG |= RB_HSPI_TX_TOG_EN;

    // DUDMA 1 Enable dual DMA function Default enabled
    R8_HSPI_CFG |= RB_HSPI_DUALDMA;

    // Enable fast DMA request
    R8_HSPI_AUX |= RB_HSPI_REQ_FT;

    // TX sampling edge
    R8_HSPI_AUX |= RB_HSPI_TCK_MOD; // falling edge sampling

    // Hardware Auto ack time
    R8_HSPI_AUX &= ~RB_HSPI_ACK_TX_MOD;

    // Delay time
    R8_HSPI_AUX &= ~RB_HSPI_ACK_CNT_SEL; //  Delay 2T

    // clear ALL_CLR  TRX_RST  (reset)
    R8_HSPI_CTRL &= ~(RB_HSPI_ALL_CLR | RB_HSPI_TRX_RST);

    // Enable Interrupt
    R8_HSPI_INT_EN |= RB_HSPI_IE_T_DONE; // Single packet sending completed
    R8_HSPI_INT_EN |= RB_HSPI_IE_R_DONE; // Single packet reception completed
    R8_HSPI_INT_EN |= RB_HSPI_IE_FIFO_OV;

    // config TX customized Header
    R32_HSPI_UDF0 = 0x3ABCDEF; // UDF0
    R32_HSPI_UDF1 = 0x3456789; // UDF1

    R32_HSPI_TX_ADDR0 = (uint32_t)out_buf0;
    R32_HSPI_RX_ADDR0 = (uint32_t)in_buf0;

    R32_HSPI_TX_ADDR1 = (uint32_t)out_buf1;
    R32_HSPI_RX_ADDR1 = (uint32_t)in_buf1;

    R16_HSPI_DMA_LEN0 = DMA_Len - 1;
    R16_HSPI_DMA_LEN1 = DMA_Len - 1;

    R16_HSPI_RX_LEN0 = 0; // 0 == 4096 bytes
    R16_HSPI_RX_LEN1 = 0; // 0 == 4096 bytes

    // Burst Disabled
    R16_HSPI_BURST_CFG = 0x0000;

    // Enable HSPI  DMA
    R8_HSPI_CTRL |= RB_HSPI_ENABLE | RB_HSPI_DMA_EN;

    PFIC_EnableIRQ(HSPI_IRQn);
    PRINT("HSPI init done.\r\n");
}

void Enable_New_OUT_Transfer(int HSPI_Tx_Buf_Num)
{
    // swap packet buffers
    // note that HSPI swaps them automatically
    // starting the first transfer with out_buf0
    USBSS->UEP1_RX_DMA = (UINT32)(UINT8 *)(HSPI_Tx_Buf_Num ? out_buf1 : out_buf0);

    DBG('O');
    DBG('0' + HSPI_Tx_Buf_Num);

    // and signal USB3 we are ready to receive a new packet
    USB30_OUT_Set(ENDP_1, ACK, DEF_ENDP1_OUT_BURST_LEVEL);
    USB30_Send_ERDY(ENDP_1 | OUT, DEF_ENDP1_OUT_BURST_LEVEL);
}

void Enable_New_IN_Transfer(int HSPI_Rx_Buf_Num)
{
    USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)(HSPI_Rx_Buf_Num ? in_buf1 : in_buf0);

    DBG('I');
    DBG('0' + HSPI_Rx_Buf_Num);

    USB30_IN_Set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, 1024);
    USB30_Send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
}

int main()
{
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

    /* Configure serial debugging */
    DebugInit(UART1_BAUD);
    PRINT("CH56x HSPI Bulk Device\n");
    PRINT("System Clock=%d\r\n", FREQ_SYS);

    HSPI_Init();
    mDelaymS(500);

    PRINT("Initialize DMA buffers\r\n");

    for (int i = 0; i < 1024; i++)
    {
        *(UINT32 *)(out_buf0 + i * 4) = 0xa0000000 + i;
        *(UINT32 *)(out_buf1 + i * 4) = 0xb0000000 + i;
        *(UINT32 *)(in_buf0 + i * 4) = 0;
        *(UINT32 *)(in_buf1 + i * 4) = 0;
    }

    // USB init
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);

#ifndef NO_USB3_TIMER
    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    TMR0_TimerInit(FREQ_SYS / 2);
#endif

    USB30D_init(ENABLE);

    GPIOB_ModeCfg(GPIO_Pin_23, GPIO_Slowascent_PP_8mA);
    GPIOB_ModeCfg(GPIO_Pin_24, GPIO_Slowascent_PP_8mA);


    // enable the first USB transfer
    Enable_New_OUT_Transfer(0);

    // We want to receive the new packet on USB3 while we are transmitting
    // the last packet on HSPI.
    for (;;)
    {
        GPIOB_SetBits(GPIO_Pin_23);
        GPIOB_ResetBits(GPIO_Pin_23);
    }
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void HSPI_IRQHandler(void)
{
    // transmit
    if (R8_HSPI_INT_FLAG & RB_HSPI_IF_T_DONE)
    {                                         // Single packet sending completed
        DBG('T');
        int HSPI_Tx_Buf_Num = (R8_HSPI_TX_SC & RB_HSPI_TX_TOG) >> 4;

        Enable_New_OUT_Transfer(HSPI_Tx_Buf_Num);

        R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE; // Clear Interrupt
        GPIOB_SetBits(GPIO_Pin_24);
        GPIOB_ResetBits(GPIO_Pin_24);
    }

    // receive
    if (R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE)
    {                                         // Single packet reception completed
        DBG('R');

        // Determine whether the CRC is correct
        if (R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR)
        { // CRC check err
            DBGERR('c');
            HSPI_Rx_End_Err |= 1;
        }

        // Whether the received serial number matches, (does not match, modify the packet serial number)
        if (R8_HSPI_RTX_STATUS & RB_HSPI_NUM_MIS)
        { // Mismatch
            DBGERR('m');
            HSPI_Rx_End_Err |= 2;
        }

        // The CRC is correct, the received serial number matches (data is received correctly)
        if (!(R8_HSPI_RTX_STATUS & (RB_HSPI_CRC_ERR | RB_HSPI_NUM_MIS)))
        {
            HSPI_Rx_End_Err = 0;
            if (In_FIFO_Read_Pos == In_FIFO_Write_Pos) Enable_New_IN_Transfer(In_FIFO[In_FIFO_Read_Pos]);
            In_FIFO_Write_Pos = (In_FIFO_Write_Pos + 1) & 1;

            int HSPI_Rx_Buf_Num = (R8_HSPI_TX_SC & RB_HSPI_RX_TOG) >> 4;
            In_FIFO[In_FIFO_Write_Pos] = HSPI_Rx_Buf_Num;

            GPIOB_SetBits(GPIO_Pin_24);
            GPIOB_ResetBits(GPIO_Pin_24);
            GPIOB_SetBits(GPIO_Pin_24);
            GPIOB_ResetBits(GPIO_Pin_24);
        }

        R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE; // Clear Interrupt
    }

    if (R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV)
    {
        DBG('o');
        HSPI_Rx_End_Err |= 4;

        R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV; // Clear Interrupt
    }
}
