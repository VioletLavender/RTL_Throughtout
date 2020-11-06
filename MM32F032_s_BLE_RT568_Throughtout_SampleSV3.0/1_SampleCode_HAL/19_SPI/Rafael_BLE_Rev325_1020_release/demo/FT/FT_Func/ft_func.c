/*******************************************************************
*
*      Copyright (c) 2020, All Right Reserved
*      Rafael Microelectronics Co. Ltd.
*      Taiwan, R.O.C.
*
*******************************************************************/
#include <stdio.h>
#include <string.h>
#include "ft_func.h"
#include "rf_phy.h"
#include "porting_spi.h"
#include "porting_misc.h"
#include "porting_LLtimer.h"


/*******************************************************************************
*   CONSTANT AND DEFINE
*******************************************************************************/
#define ADDED_SLEEPTEST_IN_TXTEST           1 // added RTC sleep/ wakeup 5 times test into TX test case


#define REG_START                           8
#define REG_DCDC_LDO_CONTROL                40

#define TEST_REG_LEN                        256
#define TEST_WRITE_REG_LEN                  240   // Write R8-R247

#define TEST_COMMAND_DUMMY                  "DUMMY\n"
#define TEST_COMMAND_DUT_TXTEST             "DUTTXTEST\n"
#define TEST_COMMAND_DUT_RXTEST             "DUTRXTEST\n"


/** TX Registers
 * @note  R40[4]=1(buck on);R40[6]=1(ldo on)
 * @note  R47[0]=0 16MHz CLK out enabled
*/
#define TEST_LDO_DCDC_TX_SYS_CURR_REG   {   0,  0,  0,  0,  0,  0,  0,  1, \
                                           28,128,222, 36,224,245, 75,  5, \
                                           33,  7,240, 32, 56,128,238,218, \
                                          143, 58,  0,128,255,255,255, 39, \
                                          163,  0,  0,210,136,236,146,154, \
                                          208, 10, 96, 67,  1, 32,106,  0, \
                                            0,  0,  0,  0,  0,192,240,  0, \
                                            0,  6,162,170,  0, 60,255,121, \
                                            0,210,249, 25, 45, 15, 30,  0, \
                                          160, 31, 85,161,  0, 68,  0, 82, \
                                            0,  6, 30, 30,  0,  6,142, 36, \
                                            1, 68,  0, 80, 80,255,  5,255, \
                                          248, 96,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                           85, 85, 85,255,255, 79,  0,  0, \
                                            4,128, 41, 65,118,113,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  5,  0,  0, 64, \
                                            0,  4,  2, 64,  0, 80,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,128 \
                                         }

/** RX Registers
 * @note  R40[4]=1(buck on);R40[6]=1(ldo on)
 * @note  R47[0]=1 16MHz CLK out disabled
*/
#define TEST_LDO_DCDC_RX_SYS_CURR_REG   {   0,  0,  0,  0,  0,  0,  0,  0, \
                                           28,128,222, 36,224,245, 75,  5, \
                                           33,  7,240, 32, 56,128,238,218, \
                                          143, 58,  0,128,255,255,255, 39, \
                                           98,  0,112,134,136,236,146,154, \
                                          208, 10, 96, 67,  1, 32,106,  1, \
                                            0,  0,  0,  0,  0,192,248,  0, \
                                            0,  6,162,170,  0, 60,255,121, \
                                            0,210,249, 25, 45, 15, 30,  0, \
                                          160, 31,248,161,  0, 68,  0, 82, \
                                            0,  6, 30, 30,  0,  6,142, 36, \
                                            1, 68,  0, 80, 80,255,  5,255, \
                                          248, 96,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                           85, 85, 85,  0,  0,  0,  0,  0, \
                                            0,128, 41, 65,118,113,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,127,255,255,255, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  5,  0,  0, 64, \
                                            0,  4,  2, 64,  0, 80,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,128 \
                                      }



/** RSSI Test --> TX Registers
 * @note  LDO, +10dBm
 * @note  R47[0]=1 16MHz CLK out disabled
*/
#define TEST_RSSI_TX_REG                {   0,  0,  0,  0,  0,  0,  0,  1, \
                                           28,128,222, 36,224,245, 75,  5, \
                                           33,  7,240, 32, 56,128,238,218, \
                                          143, 58,  0,128,255,255,255, 39, \
                                           98,  0, 32,210,136,236,146,154, \
                                          192, 10, 96, 67,  1, 32,106,  1, \
                                            0,  0,  0,  0,  0,192,240,  0, \
                                            0,  6,162,170,  0,  0,255,121, \
                                            0,210,249, 25, 45, 15, 30,  0, \
                                          160, 31, 85,161,  0, 68,  0, 82, \
                                            0,  6, 30, 30,  0,  6,142, 36, \
                                            1, 68,  0, 80, 80,255,  5,255, \
                                          248, 96,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                           85, 85, 85,255,255, 79,  0,  0, \
                                            4,128, 41, 65,118,113,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  5,  0,  0, 64, \
                                            0,  4,  2, 64,  0, 80,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,128 \
                                          }


/** RSSI Test --> RX Registers
 * @note  LDO
 * @note  R47[0]=1 16MHz CLK out disabled
*/
#define TEST_RSSI_Rx1_REG               {   0,  0,  0,  0,  0,  0,  0,  1, \
                                           28,128,222, 36,224,245, 75,  5, \
                                           33,  7,240, 32, 56,128,238,218, \
                                          143, 58,  0,128,255,255,255, 39, \
                                           98,  0,112,134,136,236,114,119, \
                                          192, 10, 96, 67,  1, 32,106,  1, \
                                            0,  0,  0,  0,  0,192,248,  0, \
                                            0,  6,162,170,  0,  0,255,121, \
                                            0,210,249, 25, 45, 15, 30,  0, \
                                          160, 31,248,161,  0, 68,  0, 82, \
                                            0,  6, 30, 30,  0,  6,142, 36, \
                                            1, 68,  0, 80, 80,255,  5,255, \
                                           17, 96,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                           85, 85, 85,  0,  0,  0,  0,128, \
                                            0, 64, 41, 65,118,113,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,127,255,255,255, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  5,  0,  0, 64, \
                                            0,  4,  2, 64,  0, 80,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0  ,0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                         }

/** 16MHz CLK Test Registers
 * @note  LDO
 * @note  R47[0]=1 16MHz CLK out enabled
*/
#define TEST_LDO_16MCLK_OUT_REG      {     0,  0,  0,  0,  0,  0,  0,  1, \
                                           28,128,222, 36,224,245, 75,  5, \
                                           33,  7,240, 32, 56,128,238,218, \
                                          143, 58,  0,128,255,255,255, 39, \
                                           98,  0,112,138,136,236,146,154, \
                                          192, 10, 96, 67,  1, 32,106,  0, \
                                            0,  0,  0,  0,  0,128,240,  0, \
                                            0,  6,162,170,  0, 60,255,121, \
                                            0,210,249, 25, 45, 15, 30,  0, \
                                          160, 31, 85,161,  0, 68,  0, 82, \
                                            0,  6, 30, 30,  0,  6,142, 36, \
                                            1, 68,  0, 80, 80,255,  5,255, \
                                          248, 96,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                           85, 85, 85,255,255, 79,  0,  0, \
                                            4,128, 41, 65,118,113,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  5,  0,  0, 64, \
                                            0,  4,  2, 64,  0, 80,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,  0, \
                                            0,  0,  0,  0,  0,  0,  0,128 \
                                       }





// Test Register Scripts
const Uint8 Test_LDODCDC_TxSysCurr_Reg[TEST_REG_LEN] = TEST_LDO_DCDC_TX_SYS_CURR_REG;
const Uint8 Test_LDODCDC_RxSysCurr_Reg[TEST_REG_LEN] = TEST_LDO_DCDC_RX_SYS_CURR_REG;
const Uint8 Test_LDO_RssiTxSysCurr_Reg[TEST_REG_LEN] = TEST_RSSI_TX_REG;
const Uint8 Test_LDO_RssiRx1SysCurr_Reg[TEST_REG_LEN] = TEST_RSSI_Rx1_REG;
const Uint8 Test_LDO_16MCLK_Reg[TEST_REG_LEN] = TEST_LDO_16MCLK_OUT_REG;


/*******************************************************************************
*   LOCAL VARIABLES
*******************************************************************************/
Sint8 dongleRssi = 0;

/*******************************************************************************
*   LOCAL FUNCTIONS
*******************************************************************************/
static void RT568_SetCmd(uint32_t u32SrcAddr);
static void RT568_SetRssiTestRx(void);
static Sint8 RT568_GetRssiValue(void);
static Uint8 RT568_CheckRssiValue(Sint8 rssiValue,Sint8 rssiBaseValue, Uint8 rssiRange);
static RT568FT_TestStatus RT568_SleepWakeupTest(uint8_t sleep_ms);

/*******************************************************************************
*   GLOBAL FUNCTIONS
*******************************************************************************/

/** TestCase: TX at 2402MHz Test
 * @note  Frequency is 2402MHz
*/
RT568FT_TestStatus RT568_TxTest(RT568VoltageMode mode)
{
    uint8_t regData;

#if (ADDED_SLEEPTEST_IN_TXTEST == 1 )
    if(RT568_SleepWakeupTest(5) != RT568_TEST_OK) // 5ms
    {
        return RT568_TEST_ERROR_SLEEP;
    }
#else
    // initialize RF PHY
    RF_Init();

    // diable sleep mode
    RF_External_Wakeup();

    // disable all RT568 interrupts
    RF_IntReset();

#endif

    // disable MCU interrupts
    MCU_GpioIntDisable();

    //R40[4]=1(buck on);R40[6]=1(ldo on)
    RT568_SetCmd((uint32_t)Test_LDODCDC_TxSysCurr_Reg);

    switch(mode)
    {
    case LDO_MODE:
        // disable DCDC
        regData = (Test_LDODCDC_TxSysCurr_Reg[REG_DCDC_LDO_CONTROL] & 0xEF);      //R40[4]=0(buck off)
        SPI_1BYT_SetTx(REG_DCDC_LDO_CONTROL, regData);
        break;

    case DCDC_MODE:
        // disable LDO
        regData = (Test_LDODCDC_TxSysCurr_Reg[REG_DCDC_LDO_CONTROL] & 0xBF);       //R40[6]=0(ldo off)
        SPI_1BYT_SetTx(REG_DCDC_LDO_CONTROL, regData);
        break;

    default:
        return RT568_TEST_ERROR_CMD;
    }

    Tiny_Delay(10);
    return RT568_TEST_OK;
}


/** TestCase: RX Test
*/
RT568FT_TestStatus RT568_RxTest(RT568VoltageMode mode)
{
    uint8_t regData;

    // disable MCU interrupts
    MCU_GpioIntDisable();

    RT568_SetCmd((uint32_t)Test_LDODCDC_RxSysCurr_Reg);

    switch(mode)
    {
    case LDO_MODE:
        // disable DCDC
        regData = (Test_LDODCDC_RxSysCurr_Reg[REG_DCDC_LDO_CONTROL] & 0xEF);      //R40[4]=0(buck off)
        SPI_1BYT_SetTx(REG_DCDC_LDO_CONTROL, regData);
        break;

    case DCDC_MODE:
        // disable LDO
        regData = (Test_LDODCDC_RxSysCurr_Reg[REG_DCDC_LDO_CONTROL] & 0xBF);       //R40[6]=0(ldo off)
        SPI_1BYT_SetTx(REG_DCDC_LDO_CONTROL, regData);
        break;

    default:
        return RT568_TEST_ERROR_CMD;
    }

    Tiny_Delay(10);
    return RT568_TEST_OK;
}


/* TestCase: Deep Sleep */
RT568FT_TestStatus RT568_DeepSleepTest(void)
{
    BleStackStatus status;
    extern void RF_IntReset(void);
    extern void RF_External_Wakeup(void);

    // initialize RF PHY
    RF_Init();

    // diable sleep mode
    RF_External_Wakeup();

    // disable all RT568 interrupts
    RF_IntReset();

    // disable MCU interrupts
    MCU_GpioIntDisable();

    // enter deep sleep
    status = setRF_EnterDeepSleep();

    if(status != BLESTACK_STATUS_SUCCESS)
    {
        return RT568_TEST_ERROR_DEEPSLEEP;
    }

    Tiny_Delay(10);
    return RT568_TEST_OK;
}


/* TestCase: Sleep */
RT568FT_TestStatus RT568_SleepTest(void)
{
    extern void RF_IntReset(void);
    extern void RF_External_Wakeup(void);
    extern void RF_Tmr_Periodic_initial(uint32_t period_tick, uint8_t sleep_mode);

    // initialize RF PHY
    RF_Init();

    // diable sleep mode
    RF_External_Wakeup();

    // disable all RT568 interrupts
    RF_IntReset();

    // disable MCU interrupts
    MCU_GpioIntDisable();

    // enter sleep mode
    //RF_Tmr_Periodic_initial(12000,1); // 1.5s
    RF_Tmr_Periodic_initial(80000,1); // 10s

    Tiny_Delay(10);
    return RT568_TEST_OK;
}



/* TestCase: RSSI */
RT568FT_TestStatus RT568_RssiTest(int8_t rssiBase, uint8_t rssiRange, uint32_t waitingTime_ms, RssiTestResult* rssiResult)
{
    Sint8 dutRssiVal = 0;
    static int count = 0;

    // disable MCU interrupts
    MCU_GpioIntDisable();

    // reset value
    dongleRssi = 0;

    // wait for dongle and flush garbage data
    Tiny_Delay(30000);
    UART1_SendData((Uint8 *)TEST_COMMAND_DUMMY,strlen((char *)TEST_COMMAND_DUMMY));
    Tiny_Delay(30000);
    /*=================================================================
     *  TX: start TX and wait to get RSSI value from golden dongle
      =================================================================*/
    // RT568 start TX and send command to dongle
    UART1_SendData((Uint8 *)TEST_COMMAND_DUT_TXTEST,strlen((char *)TEST_COMMAND_DUT_TXTEST));
    RT568_SetCmd((uint32_t)&Test_LDO_RssiTxSysCurr_Reg);

    Tiny_Delay(500);

    // waiting to get dongle rssi value via UART
    count = 0;
    while(dongleRssi == 0)
    {
        count++;

        if(count > waitingTime_ms)
        {
            return RT568_TEST_ERROR_TIMEOUT;
        }
        Tiny_Delay(1000);  // waiting
    }

    rssiResult->dongle_rssiValue = dongleRssi;
// D_msg("dongle RX Rssi=%d\n", dongleRssi);

    if(RT568_CheckRssiValue(dongleRssi,rssiBase,rssiRange) == 0)
    {
        return RT568_TEST_ERROR_RSSI;
    }


    /*=================================================================
     *  RX: start to get RSSI value
      =================================================================*/
    RT568_SetRssiTestRx();
    UART1_SendData((Uint8 *)TEST_COMMAND_DUT_RXTEST,strlen((char *)TEST_COMMAND_DUT_RXTEST));

    // check rssi
    Tiny_Delay(500);
    dutRssiVal = RT568_GetRssiValue();
    rssiResult->dut_rssiValue = dutRssiVal;
    //D_msg("DUT RX RSSI = %d\n",dutRssiVal);

    if(RT568_CheckRssiValue(dutRssiVal,rssiBase,rssiRange) == 0)
    {
        return RT568_TEST_ERROR_RSSI;
    }

    return RT568_TEST_OK;
}


void dongleMessgae_Handler(uint8_t *data, uint8_t dataLen, void *parms)
{
    //uint32_t i;
    if((dataLen == (strlen((char *)TEST_COMMAND_DUT_TXTEST))) &&
            (strncmp((char *)TEST_COMMAND_DUT_TXTEST,(char *)data,strlen((char *)TEST_COMMAND_DUT_TXTEST)-1) == 0))
    {
        dongleRssi = (Sint8)data[(strlen((char *)TEST_COMMAND_DUT_TXTEST)) - 1];
    }
//  else
//  {
//    D_msg("From UART1:");
//    for(i=0; i<dataLen; i++){
//      D_msg("0x%02x", *data++);
//    }
//    D_msg("\n");
//  }
}



/** TestCase: 16MHz CLK Test
*/
RT568FT_TestStatus RT568_16MCLK_Test(void)
{
    // disable MCU interrupts
    MCU_GpioIntDisable();

    RT568_SetCmd((uint32_t)Test_LDO_16MCLK_Reg);


    Tiny_Delay(10);
    return RT568_TEST_OK;
}


/*******************************************************************************
*   LOCAL FUNCTIONS
*******************************************************************************/

/** SleepWakeupTestCase: Enter Sleep Mode and Wakeup multiple times
*/
static RT568FT_TestStatus RT568_SleepWakeupTest(uint8_t sleep_ms)
{
    static int sleepCount = 0;
    extern uint8_t FT_WakeupFlag;
    extern void RF_IntReset(void);
    extern void RF_External_Wakeup(void);
    extern void RF_Tmr_Periodic_initial(uint32_t period_tick, uint8_t sleep_mode);

    // initialize RF PHY
    RF_Init();

    // diable sleep mode
    RF_External_Wakeup();

    // disable all RT568 interrupts
    RF_IntReset();

    // disable MCU interrupts
    MCU_GpioIntDisable();

    // enter sleep mode
    FT_WakeupFlag = 0;
    RF_Tmr_Periodic_initial((sleep_ms*8),1); // the unit of sleep time = 125us

    MCU_GpioIntEnable();

    // sleep test
    while(sleepCount < 5)
    {
        static uint8_t timeoutCount = 0;
        timeoutCount++;

        if(timeoutCount > 100) // 100ms
        {
            return RT568_TEST_ERROR_TIMEOUT;
        }

        if(FT_WakeupFlag == 1)
        {
            FT_WakeupFlag = 0;
            sleepCount++;
        }
        Tiny_Delay(1000);
    }

    sleepCount = 0;
    Tiny_Delay(10);
    return RT568_TEST_OK;
}


static void RT568_SetCmd(uint32_t u32SrcAddr)
{
    // start from register REG_START
    SPI_PDMA_SetTx(REG_START,u32SrcAddr+REG_START,TEST_WRITE_REG_LEN);
    SPI_PDMA_waitFinish();
}

static void RT568_SetRssiTestRx(void)
{
    RT568_SetCmd((uint32_t)&Test_LDO_RssiRx1SysCurr_Reg);

    Tiny_Delay(1000);

    SPI_1BYT_SetTx(121,0x40);   // R121[6] = 1: First T/R
    SPI_1BYT_SetTx(96, 0x11);   // R96[0] = 1: Enable Direct Test Mode
    // R96[7:4]: PA On Time
    SPI_1BYT_SetTx(119, 0x80);  // R119[7] = 1 :Mac T/R triggered when wakeup time = RTC time
    Tiny_Delay(1000);

    SPI_1BYT_SetTx(53, (SPI_1BYT_SetRx(53) & 0xBF)); // R56[6] = 0:Normal Mode
    Tiny_Delay(1000);
}

static Uint8 RT568_CheckRssiValue(Sint8 rssiValue,Sint8 rssiBaseValue, Uint8 rssiRange)
{
    // check RSSI value
    if((rssiValue > (rssiBaseValue + (Sint8)rssiRange)) || (rssiValue < (rssiBaseValue - (Sint8)rssiRange)))
    {
        // Error rssi value
        return 0;
    }

    return 1;
}

static Sint8 RT568_GetRssiValue(void)
{
    Sint8 rssiValue = 0;
    rssiValue = ((-280)+SPI_1BYT_SetRx(108));

    return rssiValue;
}


