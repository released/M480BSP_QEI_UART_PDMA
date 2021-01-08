/*************************************************************************//**

*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define _debug_log_UART_					(1)

//#define LED_R							(PH0)
//#define LED_Y							(PH1)
//#define LED_G							(PH2)

#define UART_PORT  						(UART5)
#define UART_TX_DMA_CH 				(11)
#define UART_TX_PDMA_OPENED_CH   		((1 << UART_TX_DMA_CH))

enum
{
	QEI_DOWN_COUNT = 0,
	QEI_UP_COUNT , 
		
};

typedef enum{

	flag_uart_rx = 0 ,
	flag_compare_error ,			
	flag_WDT ,	
	flag_UART_PDMA ,

	flag_SPI_LED_TX ,
	
	flag_DEFAULT	
}Flag_Index;

//volatile uint32_t BitFlag = 0;
extern volatile uint32_t BitFlag;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))




void dump_buffer(uint8_t *pucBuff, int nBytes);

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes);





/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
