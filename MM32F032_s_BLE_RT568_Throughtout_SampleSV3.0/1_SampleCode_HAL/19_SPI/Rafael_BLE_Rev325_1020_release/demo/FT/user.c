/**************************************************************************//**
 * @file     user.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate how to use LIB pre-build function to start and stop a BLE
 *           connection.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "BleAppSetting.h"
#include "ft_func.h"
#include "porting_misc.h"
#include "porting_spi.h"
#include "rf_phy.h"
#include "systick.h"

/*******************************************************************************
*   CONSTANT AND DEFINE
*******************************************************************************/
#define SLT_RSSITEST_BASE_VALUE               (-32)   // -32dBm,                        can be modified,
#define SLT_RSSITEST_RANGE                    20      // RSSITEST_BASE_VALUE +/- 20dBm, can be modified,
#define SLT_RSSITEST_WAITINGTIME_MAX_MS       200     // 100ms,                         can be modified,


// Request
#define APPREQUEST_IDLE                   0x01
#define APPREQUEST_FT_TEST                0x02

// Test Cases
typedef enum bleFT_TestCase
{
    TX_2402_LDO_TEST = 1,
    TX_2402_DCDC_TEST,
    RX_LDO_TEST,
    RX_DCDC_TEST,
    SLEEP_TEST,
    DEEPSLEEP_TEST,
    RSSI_TEST,
    CLK_16M_TEST
} BleFT_TestCase;


/*******************************************************************************
*   VARIABLES
*******************************************************************************/
uint8_t           appSystemRequest  = APPREQUEST_IDLE;
BleFT_TestCase    bleFtTestCase;

/*******************************************************************************
*   FUNCTIONS
*******************************************************************************/
RT568FT_TestStatus RT568_FTTestSelection(BleFT_TestCase testCase);

void BLEDemo_InitMessage(void)
{
    D_msg("+====================================================================+\n");
    D_msg("Press the number to start related testing.\n");
    D_msg("+====================================================================+\n");
    D_msg("1. TX @2402MHz(LDO) Test\n");
    D_msg("2. TX @2402MHz(DCDC) Test\n");
    D_msg("3. RX(LDO) Test\n");
    D_msg("4. RX(DCDC) Test\n");
    D_msg("5. Sleep Test\n");
    D_msg("6. Deep Sleep Test\n");
    D_msg("7. RSSI Test\n");
    D_msg("8. 16MHz CLK Test\n");
    D_msg("+====================================================================+\n");
}


void RT568_Reset(void)
{
    // Disable MCU GPIO interrupt
    MCU_GpioIntDisable();

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


void BleApp_Init(void)
{
    BLEDemo_InitMessage();
}


void BleApp_Main(void)
{
    if((appSystemRequest & APPREQUEST_FT_TEST ) != 0 )
    {
        RT568FT_TestStatus status;

        appSystemRequest &= ~APPREQUEST_FT_TEST;

        status = RT568_FTTestSelection(bleFtTestCase);

        if(status == RT568_TEST_OK)
        {
            D_msg("Result:PASS\n");
        }
        else
        {
            D_msg("Result:FAIL Error Code: 0x%02x\n",status);
        }

        // show test cases init message
        BLEDemo_InitMessage();
    }
}


RT568FT_TestStatus RT568_FTTestSelection(BleFT_TestCase testCase)
{
    RT568FT_TestStatus status;

    /* Reset RT568 to do continuous FT Test in this demo code
     * If FT test would power-off and power-on before each test case then this command can be removed.
    */
    RT568_Reset();

    // Start test
    switch(testCase)
    {
    case TX_2402_LDO_TEST:
        status = RT568_TxTest(LDO_MODE);
        D_msg("[RT568_FT TX @2402MHz LDO Test]\n");
        break;

    case TX_2402_DCDC_TEST:
        status = RT568_TxTest(DCDC_MODE);
        D_msg("[RT568_FT TX @2402MHz DCDC Test]\n");
        break;

    case RX_LDO_TEST:
        status = RT568_RxTest(LDO_MODE);
        D_msg("[RT568_FT RX LDO Test]\n");
        break;

    case RX_DCDC_TEST:
        status = RT568_RxTest(DCDC_MODE);
        D_msg("[RT568_FT RX DCDC Test]\n");
        break;

    case SLEEP_TEST:
        status = RT568_SleepTest();
        D_msg("[RT568_FT Sleep Test]\n");
        break;

    case DEEPSLEEP_TEST:
        status = RT568_DeepSleepTest();
        D_msg("[RT568_FT Deep Sleep Test]\n");
        break;

    case RSSI_TEST:
    {
        RssiTestResult rssiResult;
        status = RT568_RssiTest(SLT_RSSITEST_BASE_VALUE,SLT_RSSITEST_RANGE,SLT_RSSITEST_WAITINGTIME_MAX_MS, &rssiResult);
        D_msg("[RT568_FT RSSI Test] DUT RX: %d; Dongle RX: %d\n",rssiResult.dut_rssiValue, rssiResult.dongle_rssiValue);
    }
    break;

    case CLK_16M_TEST:
        status = RT568_16MCLK_Test();
        D_msg("[RT568_FT 16MHz CLK Test]\n");
        break;

    default:
        return RT568_TEST_ERROR_CMD;
    }

    if(status != RT568_TEST_OK)
    {
        return status;
    }

    return RT568_TEST_OK;
}



