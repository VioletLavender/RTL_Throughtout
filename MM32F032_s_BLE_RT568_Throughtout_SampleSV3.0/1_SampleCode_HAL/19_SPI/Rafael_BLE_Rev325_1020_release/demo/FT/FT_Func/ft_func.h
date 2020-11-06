/**************************************************************************//**
* @file       ft_func.h
* @brief      This file contains the functions of FT Test Functions.
*
*
* @defgroup ft_test FT Test Functions
* @{
* @details This file shows the all FT tes functions.
* @}
*****************************************************************************/

#ifndef __FT_FUNC_H__
#define __FT_FUNC_H__


/*******************************************************************************
*   INCLUDES
*******************************************************************************/
#include <stdint.h>
#include "mcu_definition.h"


/**
 * @ingroup ft_test
 * @defgroup ft_test_status Test Case Status Definition
 * @{
 * @brief  The definition of FT Test Status.
 * @{
 */
typedef uint8_t RT568FT_TestStatus;
#define RT568_TEST_OK                       0x00  /**< Command completed. */
#define RT568_TEST_ERROR_CMD                0x01  /**< Error command. */
#define RT568_TEST_ERROR_RSSI               0x02  /**< RSSI value out of range. */
#define RT568_TEST_ERROR_TIMEOUT            0x03  /**< Test timeout. */
#define RT568_TEST_ERROR_SLEEP              0x04  /**< Sleep test failed. */
#define RT568_TEST_ERROR_DEEPSLEEP          0x05  /**< Deep sleep test failed. */
/** @} */
/** @} */


/**
 * @ingroup ft_test
 * @defgroup ft_test_voltage LDO/DCDC Voltage Definition
 * @{
 * @brief  The definition of LDO and DCDC.
 * @{
 */
typedef enum rt568VoltageMode
{
    LDO_MODE,   /**< LDO. */
    DCDC_MODE,  /**< DCDC. */
} RT568VoltageMode;
/** @} */
/** @} */


/**
 * @ingroup ft_test
 * @defgroup ft_test_rssi RSSI Test Result Definition
 * @{
 * @brief  The definition of RSSI test.
 * @{
 */
typedef struct rssiTestResult
{
    signed char dongle_rssiValue;    /**< RSSI got from golden dongle. */
    signed char dut_rssiValue;       /**< RSSI got from DUT. */
} RssiTestResult;
/** @} */
/** @} */

/*******************************************************************************
*   GLOBAL FUNCTIONS
*******************************************************************************/

/** TestCase: TX at 2402MHz Test
 * @param[in] mode : voltage mode, LDO_MODE or DCDC_MODE
 * @note  Frequency is at 2402MHz
 *
 * @retval RT568_TEST_ERROR_CMD : Error voltage mode.
 * @retval RT568_TEST_OK : Command success.
*/
RT568FT_TestStatus RT568_TxTest(RT568VoltageMode mode);


/** TestCase: RX Test
 * @param[in] mode : voltage mode, LDO_MODE or DCDC_MODE
 *
 * @retval RT568_TEST_ERROR_CMD : Error voltage mode.
 * @retval RT568_TEST_OK : Command success.
*/
RT568FT_TestStatus RT568_RxTest(RT568VoltageMode mode);


/** TestCase: Deep Sleep Test
 *
 * @retval RT568_TEST_ERROR_DEEPSLEEP : Enter seep sleep mode error.
 * @retval RT568_TEST_OK : Command success.
*/
RT568FT_TestStatus RT568_DeepSleepTest(void);


/** TestCase: Sleep Test
 *
 * @retval RT568_TEST_OK : Command success.
*/
RT568FT_TestStatus RT568_SleepTest(void);


/** TestCase: RSSI Test
 *
 * @param[in] rssiBase : set RSSI checking base value.
 * @param[in] rssiRange : set RSSI checking range.
 * @param[in] waitingTime_ms : waiting time for get golden dongle RSSI value, the unit is ms.
 * @param[out] rssiResult : return dongle RX and DUT RX RSSI values.
 *
 * @retval RT568_TEST_ERROR_TIMEOUT : Get RSSI value timeout.
 * @retval RT568_TEST_ERROR_RSSI : RSSI value out of range.
 * @retval RT568_TEST_OK : Command success.
*/
RT568FT_TestStatus RT568_RssiTest(int8_t rssiBase, uint8_t rssiRange, uint32_t waitingTime_ms, RssiTestResult* rssiResult);


/** TestCase: 16MHz CLK Test
 *
 * @retval RT568_TEST_OK : Command success.
*/
RT568FT_TestStatus RT568_16MCLK_Test(void);



/** Function: handle massage from golden dongle
 *
*/
void dongleMessgae_Handler(uint8_t *data, uint8_t dataLen, void *parms);

#endif // __FT_FUNC_H__

