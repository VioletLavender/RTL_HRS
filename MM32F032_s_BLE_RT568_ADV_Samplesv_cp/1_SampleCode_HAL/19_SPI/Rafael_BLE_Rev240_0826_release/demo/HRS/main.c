/**************************************************************************//**
 * @file     main.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate BLE operation.
 *           Includes the basic initialization and the loop for kernel(BLE) operations.
 * @note
 *
 ******************************************************************************/
#define __MAIN_C__


#include <stdio.h>
#include "main.h"
#include "rf_phy.h"
#include "porting_spi.h"
#include "porting_misc.h"
#include "porting_LLtimer.h"
#include "systick.h"
#include "uart.h"
#include "spi.h"
#include "mcu_definition.h"

// use UART for TRSPX
#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
#include "host.h"   //put UART char to att_HDL_UDF01S_UDATN01 and notify_send

int BLEDemo_UartRxData_Handle(uint8_t *data, uint32_t dataLen);
void UART02_IRQHandler(void);
void UART_TX_Send(uint32_t len, uint8_t *ptr);        //show data on UART
extern void trspx_send(uint8_t *data, uint16_t len);  //send out RF data

#define TRSPX_DEFAULT_MTU   23
uint32_t TRSPX_mtu = TRSPX_DEFAULT_MTU - 3;      //transparent data block length. att_HDL_UDF01S_UDATN01[] is also 20 bytes now!!!

#endif //#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)

#pragma push
//#pragma Otime
#pragma Ospace

void RF_Open()
{
    void SPI_DMA_25xxTest(void) ;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Wait RF PHY stable ,delay 25ms */
    SysTick_DelayMs(25);


    /* RF_RESET_CLOCK  enable */
    RCC_AHBPeriphClockCmd(RF_RESET_CLOCK, ENABLE);
    /* Configure RF_RESET_PIN in output pushpull mode */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_RESET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RF_RESET_PORT, &GPIO_InitStructure);

    /* Reset RF by using RF_RESET_PIN */
    GPIO_SetBits(RF_RESET_PORT, RF_RESET_PIN);
    SysTick_DelayMs(1);
    GPIO_ResetBits(RF_RESET_PORT, RF_RESET_PIN);
    SysTick_DelayMs(1);
    GPIO_SetBits(RF_RESET_PORT, RF_RESET_PIN);


    SysTick_DelayMs(50);

    /* SPI IO remapping */
    RF_SpiIoMapping();

    /* Initialize RF PHY */
    RF_Init();                   //EnableGpioInterrupt in the end of this function

}


/*!
   \brief main loop for initialization and BLE kernel
*/
int main(void)
{
    BLE_Addr_Param bleAddrParam;

    extern BleStackStatus Ble_Kernel_Root(void);
    extern void BleApp_Main(void);
    extern void BleApp_Init(void);

#ifdef _HW_PRG_RESET_
#ifdef _HCI_HW_
    uint8_t RSP_HCI_RESET[] = {0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};
#endif
#endif  //(#ifdef _HW_PRG_RESET_)
    SysTick_Init(1000);

    NVIC_SetPriority(EXTI4_15_IRQn, 0);  //set GPIO_INT highest priority

    /* Config UART1 with parameter(115200, N, 8, 1) for printf */
    UARTx_Configure(DEBUG_UART, 115200, UART_WordLength_8b, UART_StopBits_1,  \
                    UART_Parity_No);

#ifndef _HCI_HW_
    //UART_Open(UART0, 115200);
#endif

    /* Enable the BLE RF PHY */
    RF_Open();

    /* Open UART0 for debug */
#ifndef _HCI_HW_
    printf("-------------------\n");
    printf("  BLE Start.....\n");
    printf("-------------------\n");

    printf("Chip_ID=0x%x\n",ChipId_Get());
#endif

    /* Set BD ADDR */
    bleAddrParam.addrType = PUBLIC_ADDR;
    bleAddrParam.addr[0] = 0x11;
    bleAddrParam.addr[1] = 0x22;
    bleAddrParam.addr[2] = 0x33;
    bleAddrParam.addr[3] = 0x54;
    bleAddrParam.addr[4] = 0x55;
    bleAddrParam.addr[5] = 0x56;
    setBLE_BleDeviceAddr(&bleAddrParam);


#ifdef _HW_PRG_RESET_
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();
#ifdef _HCI_HW_
        setUART_Tx(RSP_HCI_RESET, 7);
#endif
    }
#endif  //(#ifdef _HW_PRG_RESET_)

#ifndef _HCI_HW_
    /* Initial BLE App */
    BleApp_Init();
#endif

    while(1)
    {
        /* Run BLE kernel, the task priority is LL > Host */
        if(Ble_Kernel_Root() == BLESTACK_STATUS_FREE)
        {
            BleApp_Main();

#if (BLE_DEMO!=DEMO_TRSPX_UART_SLAVE)
            /* System enter Power Down mode & wait interrupt event. */
//            System_PowerDown();
#endif
        }
    }
}
#pragma pop

//Demo application - Trspx UART slave
#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)

//show data on UART
void UART_TX_Send(uint32_t len, uint8_t *ptr)
{
    uint32_t i;

    for(i=0; i<len; i++)
    {
        UART_WRITE(UART0, *ptr++);
        UART_WAIT_TX_EMPTY(UART0);
    }

    //add a new line
    UART_WRITE(UART0, 0x0D);
    UART_WRITE(UART0, 0x0A);
    UART_WAIT_TX_EMPTY(UART0);
}

//received UART data ISR: send out data by RF
/* Be careful that Central device should enable NOTIFY */
static uint8_t uartBuffer[128];

void UART02_IRQHandler(void)
{
    static uint32_t index = 0u;
    uint8_t volatile uartReceiveByte;

    uint32_t u32IntSts = UART0->INTSTS;
    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        while(UART_IS_RX_READY(UART0))
        {
            uartReceiveByte = UART_READ(UART0);
            uartBuffer[index] = uartReceiveByte;

            index++;

            if(index >= TRSPX_mtu)
            {
                trspx_send(uartBuffer,index);           //Send out UART data by RF
                index = 0;
            }
            else if((uartBuffer[index - 1] == '\r') || (uartBuffer[index - 1] == '\n'))
            {
                if(index > 1)
                {
                    uint32_t length = (uint32_t)index - 1; //Remove '\r' or '\n'
                    trspx_send(uartBuffer,length);         //Send out UART data by RF
                }
                index = 0;
            }
        }
    }
}


#endif  //#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)



