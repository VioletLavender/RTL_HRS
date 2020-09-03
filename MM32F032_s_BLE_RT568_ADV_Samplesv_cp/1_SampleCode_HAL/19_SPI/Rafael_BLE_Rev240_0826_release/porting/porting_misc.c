/******************************************************************************
 * @file    bsp_irq_gpio.c
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
#define __PORTING_MISC_C__


/* Includes -----------------------------------------------------------------*/
#include "porting_misc.h"
#include "porting_spi.h"
#include "porting_LLtimer.h"
#include "systick.h"


/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/

/******************************************************************************
 * @brief    Assign GPIO INT pin
 * @param       
 * @retval      
 * @attention  
******************************************************************************/
void MCU_GpioPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* RF_IRQ_CLOCK enable */
    RCC_AHBPeriphClockCmd(RF_IRQ_CLOCK, ENABLE);
    /*Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_IRQ_PIN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_FLOATING ;//GPIO_Mode_IPD ; //GPIO_Mode_FLOATING
    GPIO_Init(RF_IRQ_PORT, &GPIO_InitStructure);  
}

void MCU_GpioIntEnable(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
    // Configure PB8 as Input mode and enable interrupt by rising edge trigger
    MCU_GpioPinInit();      
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//IRQ - pb8
	SYSCFG_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void MCU_GpioIntDisable(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	     
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//IRQ - pb8
	SYSCFG_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/* GPIO Interrupt Handler */

//void EXTI4_15_IRQHandler(void)
//{
//    extern void LL_GPIO_Isr(void);
//    
//    if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
//    {
//        EXTI_ClearITPendingBit(EXTI_Line8);  
//        LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
//    }   
////    LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
//}

void EXTI4_15_IRQHandler(void)
{
    extern void LL_GPIO_Isr(void);
    extern void delay_us(u16 us) ;
    if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line8);
//        LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
        while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 1)  
        {
            LL_GPIO_Isr(); 
            //Tiny_Delay(1);
//            LL_GPIO_Isr(); 
        }
        //EXTI_ClearITPendingBit(EXTI_Line8);
    }     
}

void spiGpioWriteReg(const unsigned char regAddr, const unsigned char regData)
{

    unsigned char SPICount;                               // Counter used to clock out the data
    unsigned char SPIData;                                // Define a data structure for the SPI data.

    SPI_CS = 1;                                           // Make sure we start with /CS high
    SPI_CK = 0;                                           // and CK low

    SPI_CS = 0;                                           // Set /CS low to start the SPI cycle 25nS
    // Although SPIData could be implemented as an "int", resulting in one
    // loop, the routines run faster when two loops are implemented with
    // SPIData implemented as two "char"s.

    delay_us(1);

    //Address 1th byte
    SPIData = regAddr & 0x7F;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        delay_us(1);                                    // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }                                                     // and loop back to send the next bit
    // Repeat for the Data byte
    //Address 2nd byte
    SPIData = (regAddr & 0x80) >> 7;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        delay_us(1);                                     // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }

    //Data
    SPIData = regData;                                    // Preload the data to be sent with Data
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Data
    {
        if (SPIData & 0x80)
            SPI_MOSI = 1;
        else
            SPI_MOSI = 0;

        delay_us(1);
        SPI_CK = 1;
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;
    }

    delay_us(1);

    SPI_CS = 1;
    SPI_MOSI = 0;
}

/******************************************************************************
 * @brief    SPI IO mapping. Must do this after Power ON
 * @param
 * @retval
 * @attention
******************************************************************************/
void SPI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOB Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RF_IRQ_CLOCK, ENABLE);
    /*Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_IRQ_PIN | GPIO_Pin_12 | GPIO_Pin_13 |  \
                                    GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(RF_IRQ_PORT, &GPIO_InitStructure);
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 * @attention  SPI IO mapping. Must do this after Power ON
******************************************************************************/
void RF_SpiIoMapping(void)
{
    /*
     *------------------------------------------------
     *          NSS     SCK     MISO    MOSI   IRQ  AF
     *------------------------------------------------
     *   SPI2   PB12    PB13    PB14    PB15   PB8  AF0
     *------------------------------------------------
     */
    /*(1) Here set all CLK,CS,MOSI,NISO,INT as output */
    SPI_GPIO_Init();
    SPI_CS = 1;
    SPI_CK = 0;
    SPI_MOSI = 0;
    SPI_MISO = 0;
    DEFAULT_INT = 0;
//    GPIO_SetBits(GPIOB, GPIO_Pin_12);  /* SPI_CS = 1 ;*/
//    GPIO_ResetBits(GPIOB, GPIO_Pin_13); /* SPI_CK=0; */
//    GPIO_ResetBits(GPIOB, GPIO_Pin_15); /* SPI_MOSI=0; */
//    GPIO_ResetBits(GPIOB, GPIO_Pin_14); /* SPI_MISO=0; */
//    GPIO_ResetBits(RF_IRQ_PORT, RF_IRQ_PIN); /* DEFAULT_INT=0; */

    //(2) Write R248, R249 GPIO select
    //Write R248=8'b10,001,000, R249=8'b0,100,011,0
    //GPIO0[2:0]=0 - CS
    //GPIO1[2:0]=1 - CLK
    //GPIO2[2:0]=2 - MOSI
    //GPIO3[2:0]=3 - MISO
    //GPIO4[2:0]=4 - INT
    delay_us(25);
    spiGpioWriteReg(248, 0x88);
    delay_us(25);
    spiGpioWriteReg(249, 0x46);
    delay_us(25);

    /* (3) Output all pin as HIGH, last 10ms(>1ms). trigger HW take effect */
    SPI_GPIO_Init();
    SPI_CS = 1;
    SPI_CK = 1;
    SPI_MOSI = 1;
    SPI_MISO = 1;
    DEFAULT_INT = 1;
    SysTick_DelayMs(10);  //Delay 10ms

    //(4) Init GPIO & SPI
    MCU_GpioPinInit();        //Set GPIO interrupt pin as input
    /* Init SPI_BLE : SPI master, clk=12M, mode 0, 8-bit, MSB first  */
    BSP_SPI_BLE_Init();

    SPI_1BYT_SetTx(249, (0x46 | 0x80));  //set RF MISO, INT as output

    //manual control
    SPI_1BYT_SetTx(53, 0xC0); //To gurantee DC/DC power on when set R40=0x90

    //enable LDO
//    SPI_1BYT_SetTx(40, 0xC0);
    SPI_1BYT_SetTx(40, 0xC0);// 截取正常波形中是 0x90 而不是 0xc0
    SysTick_DelayMs(25);        //Put delay after LDO_enable, or set register may have strange behavior!

    //enable chip
    SPI_1BYT_SetTx(53, 0x80); //Enable chip
    SysTick_DelayMs(10);   //Put delay after chip_enable, or set register may have strange behavior!
}
/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

