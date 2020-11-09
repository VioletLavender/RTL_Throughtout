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
#include "ble_cmd.h"
//#define _PWR_test

//uint8_t temp_tx[16]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
//uint8_t temp_rx[16]={0x00};

// use UART for TRSPX
#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)
    #include "host.h"   //put UART char to att_HDL_UDF01S_UDATN01 and notify_send

    int BLEDemo_UartRxData_Handle(uint8_t *data, uint32_t dataLen);
    void UART02_IRQHandler(void);
    void UART_TX_Send(uint32_t len, uint8_t *ptr);        //show data on UART
    extern void trspx_send(uint8_t *data, uint16_t len);  //send out RF data

    #if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
        #define TRSPX_DEFAULT_MTU   247
    #else
        #define TRSPX_DEFAULT_MTU   23
    #endif
    uint32_t TRSPX_mtu = TRSPX_DEFAULT_MTU - 3;      //transparent data block length. att_HDL_UDF01S_UDATN01[] is also 20 bytes now!!!

#endif //#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)

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
bool  _PWR_Deepsleep_flag = false;
/******************************************************************************
 * @brief       Configure PB1(KEY1) GPIO and EXTI1 Line
 * @param
 * @retval
 * @attention
******************************************************************************/
void EXTI0_1_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIOB Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Selects the PB1(KEY1) used as EXTI Line */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

    /* Configure PB1(KEY1) EXTI1 Line */
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line    = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Configure PB1(KEY1) as pull down input */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable and Set PB1(KEY1) EXTI1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd    = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/******************************************************************************
 * @brief       EXTI0_1 interrupt handles
 * @param
 * @retval
 * @attention
******************************************************************************/
void EXTI0_1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        if (_PWR_Deepsleep_flag == false)
        {
            _PWR_Deepsleep_flag = true;
        }
        /* Clear the EXTI line 1 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
/*!
   \brief main loop for initialization and BLE kernel
*/

void Sys_Standby(void)
{
    // PWR CLOCK
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    //  RCC->APB2RSTR|=0X01FC;//REAST IO
    //Enable to wake up pin function
    PWR_WakeUpPinCmd(ENABLE);
    //Enter standby mode
    PWR_EnterSTANDBYMode();
}

void WKUP_STOP_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    // EXTI  SYSTEM CLOLK ENABLE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN, ENABLE);

     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;                                  //PA0,K2£¨WK_UP£©
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;                               //set as pull down input
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10 | GPIO_Pin_11;                   //PB10 K3,PB11 K4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                               //set as pull up input
    GPIO_Init(GPIOB, &GPIO_InitStructure);



    //PA.0 use EXTI line 0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //PB.10 use EXTI line 10
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);

    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //PB.11 use EXTI line 11
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);

    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

int main(void)
{
    int i;
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

    for(i=WWDG_IWDG_IRQn; i<=UART2_IRQn; i++)
    {
       NVIC_SetPriority((IRQn_Type)i,1);            //set other interrupt priority 1
    }
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

    printf("Chip_ID=0x%x\n", ChipId_Get());
#endif
/*
MCU_GpioIntDisable();
SPI_PDMA_SetTx(128, (uint32_t)&temp_tx[0], 16);
SPI_PDMA_SetRx_Isr(128, (uint32_t)&temp_rx[0], 16);
while(1);
*/
    /* Set BD ADDR */
    bleAddrParam.addrType = PUBLIC_ADDR;
    bleAddrParam.addr[0] = 0x11;
    bleAddrParam.addr[1] = 0x22;
    bleAddrParam.addr[2] = 0x33;
    bleAddrParam.addr[3] = 0x54;
    bleAddrParam.addr[4] = 0x50;
    bleAddrParam.addr[5] = 0x43;
    setBLE_BleDeviceAddr(&bleAddrParam);


#ifdef _HW_PRG_RESET_
    if (WDT_GET_RESET_FLAG() == 1)
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

    while (1)
    {
        /* Run BLE kernel, the task priority is LL > Host */
        if (Ble_Kernel_Root() == BLESTACK_STATUS_FREE)
        {
//    GPIO_SetBits(RF_TEST_PORT, RF_TEST_PIN);
            BleApp_Main();

//    SysTick_DelayMs(1);
//    GPIO_ResetBits(RF_TEST_PORT, RF_TEST_PIN);
//#if (BLE_DEMO!=DEMO_TRSPX_UART_SLAVE)
//            /* System enter Power Down mode & wait interrupt event. */
////            System_PowerDown();
//#endif
//        if (_PWR_Deepsleep_flag == false)
//        {
//            setBLE_TxPower(TX_POWER_8_DBM,STATE_BLE_ADVERTISING);

////            printf("GO TO DEEP SLEEP MODE\r\n");
////            _PWR_Deepsleep_flag = true;
////            setRF_EnterDeepSleep();

//////            Sys_Standby();

////            RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
////            RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN, ENABLE);
////            PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

//        }
        }

    }
}

//Demo application - Trspx UART slave
#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)

    ////show data on UART
    //void UART_TX_Send(uint32_t len, uint8_t *ptr)
    //{
    //    uint32_t i;

    //    for(i=0; i<len; i++)
    //    {
    //        UART_WRITE(UART0, *ptr++);
    //        UART_WAIT_TX_EMPTY(UART0);
    //    }

    //    //add a new line
    //    UART_WRITE(UART0, 0x0D);
    //    UART_WRITE(UART0, 0x0A);
    //    UART_WAIT_TX_EMPTY(UART0);
    //}

    ////received UART data ISR: send out data by RF
    ///* Be careful that Central device should enable NOTIFY */
    //static uint8_t uartBuffer[245];

    //void UART02_IRQHandler(void)
    //{
    //    static uint32_t index = 0u;
    //    uint8_t volatile uartReceiveByte;

    //    uint32_t u32IntSts = UART0->INTSTS;
    //    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    //    {
    //        while(UART_IS_RX_READY(UART0))
    //        {
    //            uartReceiveByte = UART_READ(UART0);
    //            uartBuffer[index] = uartReceiveByte;
    //            index++;
    //            if(index >= TRSPX_mtu)
    //            {
    //                trspx_send(uartBuffer,index);           //Send out UART data by RF
    //                index = 0;
    //            }
    //            else if((uartBuffer[index - 1] == '\r') || (uartBuffer[index - 1] == '\n'))
    //            {
    //                if(index > 1)
    //                {
    //                    uint32_t length = (uint32_t)index - 1; //Remove '\r' or '\n'
    //                    trspx_send(uartBuffer,length);         //Send out UART data by RF
    //                }
    //                index = 0;
    //            }
    //        }
    //    }
    //}


#endif  //#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)



