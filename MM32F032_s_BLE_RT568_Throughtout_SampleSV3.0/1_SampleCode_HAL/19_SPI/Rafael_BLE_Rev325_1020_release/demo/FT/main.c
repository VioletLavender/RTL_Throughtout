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
#include "bleAppSetting.h"
/*!
   \brief Initial necessary peripheral on MCU.
*/

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
    extern BleStackStatus Ble_Kernel_Root(void);
    extern void BleApp_Main(void);
    extern void BleApp_Init(void);

    /* Init System Clock 48M , HCLK2 = 24M */
    /* Init SysTick timer 1ms for SysTick_DelayMs */
    SysTick_Init(1000);

    NVIC_SetPriority(EXTI4_15_IRQn, 0);  //set GPIO_INT highest priority

    /* Config UART1 with parameter(115200, N, 8, 1) for printf */
    UARTx_Configure(DEBUG_UART, 115200, UART_WordLength_8b, UART_StopBits_1,  \
                    UART_Parity_No);
    /* Enable the BLE RF PHY */
    RF_Open();

    D_msg("-------------------\n");
    D_msg("  FT Test          \n");
    D_msg("-------------------\n");

    D_msg("Chip_ID=0x%x\n",ChipId_Get());

    BleApp_Init();

    while(1)
    {
        /* Run BLE kernel, the task priority is LL > Host */
        if(Ble_Kernel_Root() == BLESTACK_STATUS_FREE)
        {
            BleApp_Main();
        }
    }
}
