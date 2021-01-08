/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
/*_____ I N C L U D E S ____________________________________________________*/

#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include	"project_config.h"

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/

volatile uint32_t BitFlag = 0;
//#define BitFlag_ON(flag)							(BitFlag|=flag)
//#define BitFlag_OFF(flag)							(BitFlag&=~flag)
//#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
//#define ReadBit(bit)								(uint32_t)(1<<bit)

//#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
//#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

/*_____ F U N C T I O N S ______------______________________________________*/
void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
			#if (_debug_log_UART_ == 1)	//debug
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			#endif
			set_flag(flag_compare_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_compare_error))
	{
		#if (_debug_log_UART_ == 1)	//debug
    	printf("%s finish \r\n" , __FUNCTION__);	
		#endif
		set_flag(flag_compare_error , DISABLE);
	}

}

void reset_buffer(uint8_t *pucBuff, int nBytes)
{
	#if 1
    uint16_t i = 0;	
    for ( i = 0; i < nBytes; i++)
    {
        pucBuff[i] = 0x00;
    }	
	#else	//extra 20 bytes , with <string.h>
	memset(pucBuff, 0, nBytes * (sizeof(pucBuff[0]) ));
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void UART_TX_PDMA(uint8_t* Datain , uint16_t len)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	

    PDMA_Open(PDMA, UART_TX_PDMA_OPENED_CH);

	//TX
    PDMA_SetTransferCnt(PDMA,UART_TX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART_TX_DMA_CH, (uint32_t) (&Datain[0]), PDMA_SAR_INC, (uint32_t) &UART5->DAT , PDMA_DAR_FIX);//UART5_BASE
    /* Set request source; set basic mode. */
	
    PDMA_SetTransferMode(PDMA,UART_TX_DMA_CH, PDMA_UART5_TX, FALSE, 0);

    PDMA_SetBurstType(PDMA,UART_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[UART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

	UART_PDMA_ENABLE(UART5,UART_INTEN_TXPDMAEN_Msk);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & UART_TX_PDMA_OPENED_CH) == UART_TX_PDMA_OPENED_CH)
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA , UART_TX_PDMA_OPENED_CH);
				UART_PDMA_DISABLE(UART5,UART_INTEN_TXPDMAEN_Msk);				
                break;
            }
        }
        /* Check the DMA transfer abort interrupt flag */
        if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
        {
            /* Get the target abort flag */
            u32Abort = PDMA_GET_ABORT_STS(PDMA);
            /* Clear the target abort flag */
            PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
            break;
        }		
    }
}


void QEI0_Process(void)
{
	/*
		DOWN_COUNT : 0
		UP_COUNT : 1
	*/
	char str[32];
	uint8_t dir = QEI_GET_DIR(QEI0);
	uint16_t value = QEI_GET_CNT_VALUE(QEI0);

	if (is_flag_set(flag_UART_PDMA))
	{
		set_flag(flag_UART_PDMA , DISABLE);

		sprintf(str, "Encoder:%d", value);

		UART_TX_PDMA((uint8_t*)str,strlen(str));
		
		#if (_debug_log_UART_ == 1)	//debug
	    printf("%s : %5d , 0x%2X (%s , %2d , %2d)\r\n" , __FUNCTION__ ,value,dir , str , strlen(str) , sizeof(str));
		#endif	
	}
}

void QEI0_Init(void)
{
	QEI_Open(QEI0, QEI_CTL_X4_FREE_COUNTING_MODE, 0xFFFF);

    QEI_Start(QEI0);
}

//void PDMA_IRQHandler(void)
//{
//    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
//	
//    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
//    {
//        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
//    }
//    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
//    {
//        if((PDMA_GET_TD_STS(PDMA) & (1 << SPI_LED_TX_DMA_CH)) == (1 << SPI_LED_TX_DMA_CH))
//        {
//            /* Clear PDMA transfer done interrupt flag */
//            PDMA_CLR_TD_FLAG(PDMA, (1 << SPI_LED_TX_DMA_CH));

//			//insert process
//            SPI_DISABLE_TX_PDMA(SPI_LED_PORT);

//			set_flag(flag_SPI_LED_TX , ENABLE);
//        }        		
//    }
//    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
//    {
//        PDMA_CLR_TMOUT_FLAG(PDMA,SPI_LED_TX_DMA_CH);

//    }
//    else
//    {

//    }	
//}

void UARTx_Process(void)
{
	uint8_t res = 0;

	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
	
			case '1':

				break;	

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART5_Init(void)
{
    UART_Open(UART_PORT, 115200);
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif
}

void TMR1_IRQHandler(void)
{
	static uint16_t CNT_1000ms = 0;	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT_1000ms++ > 1000)
		{		
			CNT_1000ms = 0;
			#if (_debug_log_UART_ == 1)	//debug			
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
			#endif		
		
			set_flag(flag_UART_PDMA , ENABLE);

		}
		
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void Custom_Init(void)
{	
	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);

	//EVM button
	GPIO_SetMode(PG,BIT15,GPIO_MODE_INPUT);	
	
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LIRCSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    CLK_EnableModuleClock(UART5_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_HIRC, CLK_CLKDIV4_UART5(1));

    CLK_EnableModuleClock(PDMA_MODULE);

	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
	
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(QEI0_MODULE);
	
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk  );
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_UART5_TXD );

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk );
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_QEI0_A | SYS_GPA_MFPL_PA3MFP_QEI0_B );
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{	
    SYS_Init();

	UART0_Init();
	Custom_Init();	

	UART5_Init();

	TIMER1_Init();

//    NVIC_EnableIRQ(PDMA_IRQn);

	QEI0_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
		QEI0_Process();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
