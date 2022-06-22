/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/12/23
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56xusb30_LIB.h"
#include "CH56x_usb30.h"
#include "CH56x_usb20.h"
#include "CH56x_uart.h"

#include "zforth.h"

/* Global define */
#define  FREQ_SYS       96000000
//#define  FREQ_SYS     120000000
#define  UART1_BAUD     115200

/* Global Variables */
static char buf[64];

/* Function declaration */
extern int forth_main();

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     Initializes the UART1 peripheral.
 *
 * @param     baudrate: UART1 communication baud rate.
 *
 * @return    None
 */
void DebugInit(UINT32 baudrate)
{
    UINT32 x;
    UINT32 t = FREQ_SYS;
    x = 10 * t * 2 / 16 / baudrate;
    x = ( x + 5 ) / 10;
    R8_UART1_DIV = 1;
    R16_UART1_DL = x;
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R32_PA_SMT |= (1<<8) |(1<<7);
    R32_PA_DIR |= (1<<8);
}

extern UINT8 in_buf0[4096];
extern UINT8 in_buf1[4096];
extern UINT8 out_buf0[4096];
extern UINT8 out_buf1[4096];

//////////////////////////////////////// HSPI ////////////////////////////////////
//Mode
#define Host_MODE    0
#define Slave_MODE   1
/* HSPI Mode Selection */
#define HSPI_MODE   Slave_MODE

//Data size
#define DataSize_8bit   0
#define DataSize_16bit   1
#define DataSize_32bit   2
/* HSPI Data Size Selection */
#define Data_Size   DataSize_32bit // Not stable

//DMA_Len
#define DMA_Len0   4095
#define DMA_Len1   4095

/* Shared variables */
volatile int HSPI_Tx_End_Flag = 0;  // Send completion flag
volatile int HSPI_Rx_End_Flag = 0;  // Receive completion flag
volatile int HSPI_Rx_End_Err = 0;   // 0=No Error else >0 Error code
volatile int HSPI_Rx_Buf_Num = 0;
volatile int HSPI_Tx_Buf_Num = 0;

void HSPI_GPIO_Init(void)
{
    // TX GPIO PA9/PA11/PA21 Push-pull output
    R32_PA_DIR |= (1<<9) | (1<<11) | (1<<21);

    // clk 16mA
    R32_PA_DRV |= (1<<11);

    // RX GPIO PA10 Push-pull output
    R32_PA_DIR |= (1<<10);
}

/*******************************************************************************
 * @fn       HSPI_Init
 *
 * @brief    HSPI ��ʼ��
 *
 * @return   None
 */
void HSPI_Init(void)
{
    // GPIO Cfg
    HSPI_GPIO_Init();

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

    //Enable fast DMA request
    R8_HSPI_AUX |= RB_HSPI_REQ_FT;

    //TX sampling edge
    R8_HSPI_AUX |= RB_HSPI_TCK_MOD;  // falling edge sampling

    //Hardware Auto ack time
    R8_HSPI_AUX &= ~RB_HSPI_ACK_TX_MOD;

    //Delay time
    R8_HSPI_AUX &= ~RB_HSPI_ACK_CNT_SEL;   //  Delay 2T

    //clear ALL_CLR  TRX_RST  (reset)
    R8_HSPI_CTRL &= ~(RB_HSPI_ALL_CLR|RB_HSPI_TRX_RST);

    //Enable Interrupt
    R8_HSPI_INT_EN |= RB_HSPI_IE_T_DONE;  // Single packet sending completed
    R8_HSPI_INT_EN |= RB_HSPI_IE_R_DONE;  // Single packet reception completed
    R8_HSPI_INT_EN |= RB_HSPI_IE_FIFO_OV;

    //config TX customized Header
    R32_HSPI_UDF0 = 0x3ABCDEF; // UDF0
    R32_HSPI_UDF1 = 0x3456789; // UDF1

    R32_HSPI_TX_ADDR0 = (uint32_t)out_buf0;
    R32_HSPI_RX_ADDR0 = (uint32_t)in_buf0;

    R32_HSPI_TX_ADDR1 = (uint32_t)out_buf1;
    R32_HSPI_RX_ADDR1 = (uint32_t)in_buf1;

    R16_HSPI_DMA_LEN0 = DMA_Len0 - 1;
    R16_HSPI_DMA_LEN1 = DMA_Len1 - 1;;

    //Enable HSPI  DMA
    R8_HSPI_CTRL |= RB_HSPI_ENABLE | RB_HSPI_DMA_EN;

    PFIC_EnableIRQ(HSPI_IRQn);
    PRINT("HSPI init done.\r\n");
}

#ifdef ZFORTH
#include "zforth-dict.c"
#endif

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

    /* Configure serial debugging */
    DebugInit(UART1_BAUD);
    PRINT("CH56x HSPI Bulk Device\n");
    PRINT("System Clock=%d\r\n", FREQ_SYS);

    HSPI_Init();
	mDelaymS(1000);

    PRINT("Initialize DMA buffers\r\n");
#ifndef BUFBASED_INIT
    for(int i=0; i < 1024; i++) {
      *(UINT32*)(out_buf0 + i*4) = 0x0a000000 + i;
      *(UINT32*)(out_buf1 + i*4) = 0xb0000000 + i;
      *(UINT32*)(in_buf0 + i*4)  = 0;
      *(UINT32*)(in_buf1 + i*4)  = 0;
    }
#else
	for(int i=0; i<0x2000; i++){   //0x2000*4 = 32K
      *(UINT32*)(0x20020000+i*4) = i;
	}
#endif


    // USB init
#if ENABLE_USB3
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);

    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    TMR0_TimerInit(FREQ_SYS/2);

    USB30D_init(ENABLE);          //USB3.0 initialization Make sure that the two USB3.0 interrupts are enabled before initialization
#endif

    GPIOB_ModeCfg(GPIO_Pin_23, GPIO_Slowascent_PP_8mA);
    GPIOB_ModeCfg(GPIO_Pin_24, GPIO_Slowascent_PP_8mA);

#ifdef ZFORTH
    zf_init(1);
#ifndef BOOTSTRAP
    size_t len;
    unsigned char *p = (unsigned char *)zf_dump(&len);
    for (int i = 0; i < len; i++) p[i] = zforth_save[i];
#endif
    zf_bootstrap();
#ifdef BOOTSTRAP
    zf_eval(": emit 0 sys ;");
    zf_eval(": . 1 sys ;");
    zf_eval(": ! 0 !! ;");
    zf_eval(": @ 0 @@ ;");
    zf_eval(": , 0 ,, ;");
    zf_eval(": # 0 ## ;");
#endif

    /* Main loop: read words and eval */
    uint8_t l = 0;

    for(;;) {
            int c;
            while (!R8_UART1_RFC);
            c = R8_UART1_RBR;
            UART1_SendByte(c);

            if(c == '\n' || c == '\r') {
                UART1_SendByte('\r');
                UART1_SendByte('\n');
                zf_result r = zf_eval(buf);
                if (r != ZF_OK) {
                    UART1_SendString("ERR\r\n", 5);
                }
                l = 0;
            } else if (c == '\b') {
                buf[l--] = 0;
            } else if(l < sizeof(buf) - 1) {
                    buf[l++] = c;
            }

            buf[l] = '\0';
    }
#endif

    for(;;) {
    	GPIOB_InverseBits(GPIO_Pin_24);
    	mDelaymS(300);
    }
}

#ifdef ZFORTH
zf_input_state zf_host_sys(zf_syscall_id id, const char *input)
{
        char buf[16];
        int i;

        switch((int)id) {

                case ZF_SYSCALL_EMIT:
                        UART1_SendByte((char)zf_pop());
                        break;

                case ZF_SYSCALL_PRINT:
                        itoa(zf_pop(), buf, 10);
                        while (buf[i]) {
                            UART1_SendByte(buf[i++]);
                        }
                        UART1_SendByte(' ');
                        break;

                case ZF_SYSCALL_TELL: {
                    zf_cell len = zf_pop();
                    void *buf = (uint8_t *)zf_dump(NULL) + (int)zf_pop();
                    (void)fwrite(buf, 1, len, stdout);
                    fflush(stdout);
                    break;
                }
        }

        return 0;
}

zf_cell zf_host_parse_num(const char *buf)
{
        char *end;
        zf_cell v = strtol(buf, &end, 0);
        if(*end != '\0') {
            zf_abort(ZF_ABORT_NOT_A_WORD);
        }
        return v;
}

void zf_host_trace(const char *fmt, va_list va)
{
        //UART1_SendString("\033[1;30m", 7);
        vfprintf(stdout, fmt, va); fflush(stdout);
        //UART1_SendString("\033[0m", 4);
        UART1_SendByte('\r');
}
#endif

/*********************************************************************
 * @fn      HSPI_IRQHandler
 *
 * @brief   This function handles HSPI exception.
 *
 * @return  none
 */
void HSPI_IRQHandler(void)
{
	//PRINT("HSPI"); fflush(stdout);
	UART1_SendByte('H');
	UART1_SendByte('T');
	// transmit
	if(R8_HSPI_INT_FLAG & RB_HSPI_IF_T_DONE) { // Single packet sending completed
		R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE;  // Clear Interrupt
		UART1_SendByte('.');
		//PRINT(" Tx buf %d done", HSPI_Tx_Buf_Num);
		USBSS->UEP1_RX_DMA = (UINT32)(UINT8 *)(HSPI_Tx_Buf_Num ? out_buf1 : out_buf0);

		// enable USB transfer
		USB30_OUT_Set(ENDP_1, ACK, DEF_ENDP1_OUT_BURST_LEVEL);
        USB30_Send_ERDY(ENDP_1 | OUT, DEF_ENDP1_OUT_BURST_LEVEL);

        HSPI_Tx_Buf_Num = (R8_HSPI_TX_SC & RB_HSPI_TX_TOG) >> 4;
		HSPI_Tx_End_Flag = 1;
	}

    // receive
    if(R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE) {  // Single packet reception completed
        R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  // Clear Interrupt

		PRINT(" Rx ");
        // Determine whether the CRC is correct
        if(R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR){  // CRC check err
            // R8_HSPI_CTRL &= ~RB_HSPI_ENABLE;
            PRINT("CRC err");
            HSPI_Rx_End_Err |= 1;
            HSPI_Rx_End_Flag = 1;
        }

        // Whether the received serial number matches, (does not match, modify the packet serial number)
        if(R8_HSPI_RTX_STATUS & RB_HSPI_NUM_MIS){  // Mismatch
            PRINT("NUM_MIS err");
            HSPI_Rx_End_Err |= 2;
            HSPI_Rx_End_Flag = 1;
        }

        // The CRC is correct, the received serial number matches (data is received correctly)
        if( !(R8_HSPI_RTX_STATUS & (RB_HSPI_CRC_ERR | RB_HSPI_NUM_MIS)) ) {
            HSPI_Rx_End_Err = 0;
            HSPI_Rx_End_Flag = 1;

#if ENABLE_USB3
            USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)(HSPI_Rx_Buf_Num ? in_buf1 : in_buf0);
            USB30_IN_Set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, 1024);
            USB30_Send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL); // Notify the host to send 4 packets
#endif
            UART1_SendByte('0' + HSPI_Rx_Buf_Num);

            HSPI_Rx_Buf_Num = (R8_HSPI_RX_SC & RB_HSPI_RX_TOG) >> 4;
        }
    }

    if(R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV) {
        R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV; // Clear Interrupt
        PRINT("FIFO OV");
        HSPI_Rx_End_Err |= 4;
        HSPI_Rx_End_Flag = 1;
    }
}

