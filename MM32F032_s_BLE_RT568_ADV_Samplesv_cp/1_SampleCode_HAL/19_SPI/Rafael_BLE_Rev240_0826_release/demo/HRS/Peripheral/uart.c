/******************************************************************************
 * @file    uart.c
 * @author  King
 * @version V1.00
 * @date    20-May-2020
 * @brief   ......
 ******************************************************************************
 * @attention
 * 
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 * <H2><CENTER>&COPY; COPYRIGHT 2020 MINDMOTION </CENTER></H2>
******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------*/
#define __UART_C__


/* Includes -----------------------------------------------------------------*/
#include "uart.h"


bool transport_flag=false;
uint8_t UART_RX_BUF[UART_REC_LEN];
uint16_t UART_RX_STA=0; 



/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/


/******************************************************************************
 * @brief       
 * @param       
 * @retval      
 * @attention   
******************************************************************************/
void UARTx_Configure(UART_TypeDef           *UARTx,
                     uint32_t                BaudRate,
                     UART_WordLength_TypeDef WordLength,
                     UART_Stop_Bits_TypeDef  StopBits,
                     UART_Parity_TypeDef     Parity)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    if(UARTx == UART1)
    {
        /* Enable UART1 clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);

        /* UART Configuration ad follow */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.UART_BaudRate            = BaudRate;
        UART_InitStructure.UART_WordLength          = WordLength;
        UART_InitStructure.UART_StopBits            = StopBits;
        UART_InitStructure.UART_Parity              = Parity;
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
        UART_InitStructure.UART_Mode                = UART_Mode_Rx | UART_Mode_Tx;
        UART_Init(UART1, &UART_InitStructure);

        /* Enable GPIO and SYSCFG clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,    ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Enable PA9 & PA10 alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

        /* Configure UART Tx as alternate function push-pull */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

         /* Configure UART Rx as input floating */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
				
				
				UART_nvic_config();
				UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//开启串口接受中断
				
        /* Enable UART */
        UART_Cmd(UART1, ENABLE);
    }
    else if(UARTx == UART2)
    {
        /* Enable UART2 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);

        /* UART Configuration ad follow */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.UART_BaudRate            = BaudRate;
        UART_InitStructure.UART_WordLength          = WordLength;
        UART_InitStructure.UART_StopBits            = StopBits;
        UART_InitStructure.UART_Parity              = Parity;
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
        UART_InitStructure.UART_Mode                = UART_Mode_Rx | UART_Mode_Tx;
        UART_Init(UART2, &UART_InitStructure);

        /* Enable GPIO and SYSCFG clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,    ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Enable PA2 & PA3 alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

        /* Configure UART Tx as alternate function push-pull */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

         /* Configure UART Rx as input floating */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Enable UART */
        UART_Cmd(UART2, ENABLE);
    }
    else
    {
    }
}
void UART_nvic_config(void)
{

		NVIC_InitTypeDef NVIC_InitStructure;
	
    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

}

void UartSendByte(u8 dat)
{
    UART_SendData(UART1, dat);
    while(!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT));
}

void UART_TX_Send(u8 len,u8* buf)
{
    while(len--)
        UartSendByte(*buf++);
}

void UART1_IRQHandler(void)                	//串口1中断服务程序
{
    u8 Res;
    if(UART_GetITStatus(UART1, UART_IT_RXIEN)  != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
				transport_flag=true;	
        UART_ClearITPendingBit(UART1,UART_IT_RXIEN);
        Res =UART_ReceiveData(UART1);	//读取接收到的数据
        if((Res==0x0d)&&((UART_RX_STA&0X3FFF)>0))
        {
            UART_RX_STA|=0x4000;
            UART_RX_BUF[UART_RX_STA&0X3FFF]=Res ;
            UART_RX_STA++;
        }
        else if((UART_RX_STA&0x4000)&&((UART_RX_STA&0X3FFF)>0))//接收到了0x0d
        {
            if(Res==0x0a)
            {
                UART_RX_STA|=0x8000;	
															
            }
            UART_RX_BUF[UART_RX_STA&0X3FFF]=Res ;
            UART_RX_STA++;
					
        }
        else{
            UART_RX_BUF[UART_RX_STA&0X3FFF]=Res ;
            UART_RX_STA++;
            UART_RX_STA=UART_RX_STA&0X3FFF;
            if((UART_RX_STA)>(UART_REC_LEN-1))
                UART_RX_STA=0;//接收数据错误,重新开始接收	
        }
    } 
} 
/******************************************************************************
 * @brief       
 * @param       
 * @retval      
 * @attention   
******************************************************************************/
int fputc(int ch, FILE *f)
{
    /* Send a character to the UART */
    UART_SendData(DEBUG_UART, (uint8_t)ch);

     /* Loop until the end of transmission */
    while(!UART_GetFlagStatus(DEBUG_UART, UART_FLAG_TXEPT));

    return ch;
}


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

