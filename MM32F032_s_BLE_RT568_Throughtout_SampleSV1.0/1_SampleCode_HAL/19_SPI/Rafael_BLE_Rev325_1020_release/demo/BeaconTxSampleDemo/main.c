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
#include "ble_cmd.h"

#pragma push
//#pragma Otime
#pragma Ospace

void RF_Open()
{
    //void SPI_DMA_25xxTest(void) ;
    //GPIO_InitTypeDef GPIO_InitStructure;

    /* Wait RF PHY stable ,delay 25ms */
    SysTick_DelayMs(25);

    /* Initialize Gpio reset pin */
    MCU_GpioResetInit();

    /* Do Gpio Reset */
    MCU_GpioReset();
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

    /* Init System, IP clock and multi-function I/O. */
    //SYS_Init();
    /* Init System Clock 48M , HCLK2 = 24M */
    /* Init SysTick timer 1ms for SysTick_DelayMs */
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
    SysTick_DelayMs(2000);

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
    bleAddrParam.addr[1] = 0x52;
    bleAddrParam.addr[2] = 0x53;
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

            /* System enter Power Down mode & wait interrupt event. */
            System_PowerDown();
        }
    }
}
#pragma pop




