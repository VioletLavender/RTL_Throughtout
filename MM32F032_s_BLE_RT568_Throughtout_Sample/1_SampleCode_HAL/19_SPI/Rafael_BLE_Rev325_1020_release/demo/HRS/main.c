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

#if (BLE_DATA_LENGTH_EXTENSION_SUPPORT == ENABLE_)
#define TRSPX_DEFAULT_MTU   247
#else
#define TRSPX_DEFAULT_MTU   23
#endif
uint32_t TRSPX_mtu = TRSPX_DEFAULT_MTU - 3;      //transparent data block length. att_HDL_UDF01S_UDATN01[] is also 20 bytes now!!!

#endif //#if (BLE_DEMO==DEMO_TRSPX_UART_SLAVE)


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/
u8 gTxData[256];
u8 gRxData[256];
#define SectorErase           0x20
#define PP                    0x02
#define ReadData              0x03
#define ChipErase             0xC7
#define RDSR                  0x05
#define Dummy_Byte            0x00
#define W25X_BUSY             0
#define W25X_NotBUSY          1
#define FlashSize             0x400
#define ReadStatusReg         0x05


#define READ                  0x03
#define FAST_READ             0x0B
#define RDID                  0x9F
#define WREN                  0x06
#define WRDI                  0x04
#define SE                    0xD8
#define BE                    0xC7
#define PP                    0x02
#define RDSR                  0x05
#define WRSR                  0x01
#define DP                    0xB9
#define RES                   0xAB
#define GPIO_Pin_0_2  GPIO_Pin_2

void W25xx_ReadID(void)
{
    u8 temp[5];
    u32 i = 0;
    temp[i++] = RDID;

    //Spi cs assign to this pin,select
    SPI2->NSSR &= ~SPI_NSSR_NSS ;//BSP_SPI_BLE_CS_L();//

//    SPI_PDMA_SetTx(20U, ((uint32_t)(RFIP_init_reg+20)), RADIO_RF_INIT_REG_NUM+1-12);
    /* Enable SPI TX DMA function */
    SPIx_DMA_TxData(SPI2, temp, i); //u32TransCount-1UL ?
    SPIx_DMA_RxData(SPI2, &temp[i], 3);
    SPI2->NSSR |= SPI_NSSR_NSS ;//BSP_SPI_BLE_CS_H();//

}

static void W25xx_CheckStatus(void)
{
    u8 temp[5];
    u32 i = 0;
    SPI2->NSSR &= ~SPI_NSSR_NSS ;
    temp[i++] = RDSR;
    SPIx_DMA_TxData(SPI2, temp, i);
    while (1)
    {
        SPIx_DMA_RxData(SPI2, &temp[i], 1);
        if (((temp[i]) & 0x01) == 0x0)
            break;
    }
    SPI2->NSSR |= SPI_NSSR_NSS ;
}

void W25xx_WriteEnable(void)
{
    u8 temp[5];
    u32 i = 0;
    temp[i++] = WREN;
    //Spi cs assign to this pin,select
    SPI2->NSSR &= ~SPI_NSSR_NSS ;//BSP_SPI_BLE_CS_L();//

    SPIx_DMA_TxData(SPI2, temp, i);
    SPI2->NSSR |= SPI_NSSR_NSS ;//BSP_SPI_BLE_CS_H();//
}

void W25xx_SectorErase(u32 address)
{
    u8 temp[5];
    u32 i = 0;

    address = address & 0xffff0000;
    temp[i++] = SE;
    temp[i++] = ((u8)(address >> 16)) & 0xff;
    temp[i++] = ((u8)(address >> 8)) & 0xff;
    temp[i++] = ((u8)address) & 0xff;
    W25xx_WriteEnable();
    //Spi cs assign to this pin,select
    SPI2->NSSR &= ~SPI_NSSR_NSS ;
    SPIx_DMA_TxData(SPI2, temp, i);

    SPI2->NSSR |= SPI_NSSR_NSS ;
    W25xx_CheckStatus();
}

void W25xx_PageProgram(u32 address, u8 *p, u32 number)
{
    u8 temp[5];
    u32 i = 0;

    address = address & 0xffff0000;
    temp[i++] = PP;
    temp[i++] = ((u8)(address >> 16)) & 0xff;
    temp[i++] = ((u8)(address >> 8)) & 0xff;
    temp[i++] = ((u8)address) & 0xff;
    W25xx_WriteEnable();
    SPI2->NSSR &= ~SPI_NSSR_NSS ;
    SPIx_DMA_TxData(SPI2, temp, i);
    SPIx_DMA_TxData(SPI2, p, number);
    SPI2->NSSR |= SPI_NSSR_NSS ;
    W25xx_CheckStatus();
}
void W25xx_PageRead(u32 address, u8 *p, u32 number)
{

    u8 temp[5];
    u32 i = 0;
    address = address & 0xffff0000;
    temp[i++] = READ;
    temp[i++] = ((u8)(address >> 16)) & 0xff;
    temp[i++] = ((u8)(address >> 8)) & 0xff;
    temp[i++] = ((u8)address) & 0xff;
    W25xx_CheckStatus();
    //Spi cs assign to this pin,select
    SPI2->NSSR &= ~SPI_NSSR_NSS ;
    SPIx_DMA_TxData(SPI2, temp, i);
    SPIx_DMA_RxData(SPI2, p, number);
    SPI2->NSSR |= SPI_NSSR_NSS ;
}

void SPI_DMA_25xxTest(void)
{
    u32 i, result = 0;
    printf("\r\nsprintf ok\r\n");
    printf("\r\nStart SPI test\r\n");

    for (i = 0; i < 256; i++)
    {
        gTxData[i] = i * 2;
    }
    printf("SPI2 test\r\n");
    W25xx_ReadID();
    W25xx_SectorErase(0);
    W25xx_PageProgram(0, gTxData, 256);
    memset(gRxData, 0x0, 256);
    W25xx_PageRead(0, gRxData, 256);
    for (i = 0; i < 10; i++)
    {
        printf("rx[%d]=0x%x\r\n", i, gRxData[i]);
    }
    for (i = 0; i < 256; i++)
    {
        if (gTxData[i] != gRxData[i])
        {
            result = 1;
            break;
        }

    }
    if (result == 1)
    {
        printf("SPI2 WR 25xx Fail\r\n");
    }
    else
    {
        printf("SPI2 WR 25xx Successful\r\n");

    }
    printf("SPI2 test over\r\n");
}


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



