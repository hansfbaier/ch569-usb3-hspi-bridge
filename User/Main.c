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

/* Global define */
#define  FREQ_SYS       96000000
//#define  FREQ_SYS     120000000
#define  UART1_BAUD     115200
/* Global Variable */

/* Function declaration */

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
#define DMA_Tx_Len0   512
#define DMA_Tx_Len1   512

//DMA_Addr0
#define DMA_TX_Addr0   0x20020000
#define DMA_RX_Addr0   0x20020000

//DMA_Addr1
#define DMA_TX_Addr1   0x20020000 + DMA_Tx_Len0
#define DMA_RX_Addr1   0x20020000 + DMA_Tx_Len1

/* Shared variables */
volatile int Tx_End_Flag = 0;  // Send completion flag
volatile int Rx_End_Flag = 0;  // Receive completion flag
volatile int Rx_End_Err = 0; // 0=No Error else >0 Error code

/* HSPI_IRQHandler variables */
UINT32 Tx_Cnt = 0;
UINT32 Rx_Cnt = 0;
UINT32 addr_cnt = 0;

/*******************************************************************************
 * @fn      HSPI_GPIO_Init
 *
 * @brief    Initializes the HSPI GPIO.
 *
 * @return  None
 */
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
 * @brief    HSPI ³õÊ¼»¯
 *
 * @return   None
 */
void HSPI_Init(void)
{
	// GPIO Cfg
	HSPI_GPIO_Init();

	R8_HSPI_CFG &= ~(RB_HSPI_MODE | RB_HSPI_MSK_SIZE); // Clear
	R8_HSPI_CFG &= ~(RB_HSPI_MODE); // Slave

	//data size
#if (Data_Size == DataSize_8bit)
	R8_HSPI_CFG |= RB_HSPI_DAT8_MOD;

#elif (Data_Size == DataSize_16bit)
	R8_HSPI_CFG |= RB_HSPI_DAT16_MOD;

#elif (Data_Size == DataSize_32bit)
	R8_HSPI_CFG |= RB_HSPI_DAT32_MOD;

#endif

    //ACk mode 0 (Hardware auto-answer mode is used in burst mode, not in normal mode)
	R8_HSPI_CFG &= ~RB_HSPI_HW_ACK;

    // Rx ToG En  0
	R8_HSPI_CFG |= RB_HSPI_RX_TOG_EN;

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

	//Enable Interupt
	R8_HSPI_INT_EN |= RB_HSPI_IE_R_DONE;  // Single packet reception completed
	R8_HSPI_INT_EN |= RB_HSPI_IE_FIFO_OV;

	//config TX customized Header
	R32_HSPI_UDF0 = 0x3ABCDEF; // UDF0
	R32_HSPI_UDF1 = 0x3456789; // UDF1

	//addr0 DMA TX RX addr
	R32_HSPI_TX_ADDR0 = DMA_TX_Addr0;
	R32_HSPI_RX_ADDR0 = DMA_RX_Addr0;

	//addr1 DMA TX RX addr
	R32_HSPI_TX_ADDR1 = DMA_TX_Addr1;
	R32_HSPI_RX_ADDR1 = DMA_RX_Addr1;

	//addr0 DMA TX addr
	R16_HSPI_DMA_LEN0 = DMA_Tx_Len0 - 1;
	//addr1 DMA TX addr
	R16_HSPI_DMA_LEN1 = DMA_Tx_Len1 - 1;

	//Enable HSPI  DMA
	R8_HSPI_CTRL |= RB_HSPI_ENABLE | RB_HSPI_DMA_EN;

	PFIC_EnableIRQ(HSPI_IRQn);
}

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

    PRINT("Clear RAMX 0x20020000 32K\r\n");
    for(int i=0; i < 8192; i++){   // 8192*4 = 32K
      *(UINT32*)(0x20020000+i*4) = 0;
	}

    fflush(stdout);
	mDelaymS(10);

	HSPI_Init();

	// USB init
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);

    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    TMR0_TimerInit( 67000000 );   //about 0.5 seconds

	USB30D_init(ENABLE);          //USB3.0 initialization Make sure that the two USB3.0 interrupts are enabled before initialization

	GPIOB_ModeCfg( GPIO_Pin_23, GPIO_Slowascent_PP_8mA );
	GPIOB_ModeCfg( GPIO_Pin_24, GPIO_Slowascent_PP_8mA );
	while(1) {
		while(Rx_End_Flag == 0);
		GPIOB_InverseBits(GPIO_Pin_23);

        if(Rx_End_Err == 0) {
        }
        else {
			PRINT("RX error...\r\nHSPI_Init\r\n");
            HSPI_Init();
        }
	}
}

void HSPI_IRQHandler_ReInitRX(void)
{
	R32_HSPI_RX_ADDR0 = DMA_RX_Addr0;
	R32_HSPI_RX_ADDR1 = DMA_RX_Addr1;
	addr_cnt = 0;
	Rx_Cnt = 0;
}

/*********************************************************************
 * @fn      HSPI_IRQHandler
 *
 * @brief   This function handles HSPI exception.
 *
 * @return  none
 */
void HSPI_IRQHandler(void)
{
/**************/
/** Transmit **/
/**************/
	if(R8_HSPI_INT_FLAG & RB_HSPI_IF_T_DONE) {  // Single packet sending completed
		R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE;  // Clear Interrupt
	}

/*************/
/** Receive **/
/*************/
	if(R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE){  // Single packet reception completed
		R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  // Clear Interrupt

        // Determine whether the CRC is correct
        if(R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR){  // CRC check err
        	// R8_HSPI_CTRL &= ~RB_HSPI_ENABLE;
        	PRINT("CRC err\r\n");
			HSPI_IRQHandler_ReInitRX();
        	Rx_End_Err |= 1;
            Rx_End_Flag = 1;
        }

        // Whether the received serial number matches, (does not match, modify the packet serial number)
        if(R8_HSPI_RTX_STATUS & RB_HSPI_NUM_MIS){  // Mismatch
        	PRINT("NUM_MIS err\r\n");
			HSPI_IRQHandler_ReInitRX();
            Rx_End_Err |= 2;
            Rx_End_Flag = 1;
        }

        // The CRC is correct, the received serial number matches (data is received correctly)
        if( !(R8_HSPI_RTX_STATUS & (RB_HSPI_CRC_ERR | RB_HSPI_NUM_MIS)) ) {
        	Rx_Cnt++;
        	addr_cnt++;
    		if(Rx_Cnt < 64) // Receive 32K (64 * 512 bytes)
    		{
    			if(addr_cnt % 2) {
    				R32_HSPI_RX_ADDR0 += 512 * 2;
    			} else {
    				R32_HSPI_RX_ADDR1 += 512 * 2;
    			}
    		} else { // Receive completed
				HSPI_IRQHandler_ReInitRX();
    			Rx_End_Flag = 1;
    		}
        }
	}

	if(R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV) {
		R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV; // Clear Interrupt
		PRINT("FIFO OV\r\n");
		HSPI_IRQHandler_ReInitRX();
        Rx_End_Err |= 4;
        Rx_End_Flag = 1;
	}
}

