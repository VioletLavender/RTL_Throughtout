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
#include "host.h"
#include "rf_phy.h"
#include "porting_misc.h"
#include "ble_cmd.h"
#include "ble_event.h"


uint8_t connId = 0;   //Current connection Id
Uint8 ble_state = STATE_BLE_STANDBY;

/* Function prototype declaration */
static void BleEvent_Callback(BleCmdEvent event, void *param);


#pragma push
//#pragma Otime
#pragma Ospace

//Scan Parameters
#define SCAN_WINDOW                32U     //32*0.625ms=20ms
#define SCAN_INTERVAL              160U    //160*0.625ms=100ms

void BleApp_Init(void)
{
    /* register command event callback function */
    setBLE_RegisterBleEvent(BleEvent_Callback);
}

//void Ble_Reset(void)
//{
//    /* reset BLE HW/SW */
//    //MCU re-start from main()
//    SYS_UnlockReg();
//    SYS_ResetChip();
//    SYS_LockReg();
//}
/*设置广播搜索的参数并使能搜索*/
void Ble_StartScan(void)
{
    BLE_Scan_Param scanparam;
    scanparam.Scan_Type = SCAN_TYPE_ACTIVE;
    scanparam.Scan_Interval = SCAN_INTERVAL;
    scanparam.Scan_Window = SCAN_WINDOW;
    scanparam.Scan_FilterPolicy = SCAN_FILTER_POLICY_ACCEPT_ALL;

    setBLE_ScanParam(scanparam);
    setBLE_ScanEnable(SCAN_FILTER_POLICY_ACCEPT_ALL);
}

void BleApp_Main(void)
{
    if (ble_state == STATE_BLE_STANDBY)
    {
        Ble_StartScan();
        ble_state = STATE_BLE_SCANNING;
    }
    else if (ble_state == STATE_BLE_SCANNING)
    {

    }
} //end of BleApp_Main()
#pragma pop


/* GAP Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    switch (event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
        break;
    /*只要蓝牙搜索到广播就会触发此事件，我们可以在此函数中对搜索到的广播的数据进行分析处理*/
    case BLECMD_EVENT_SCAN_REPORT:
    {
        uint8_t i;
        BLE_Event_Scan_Report *rpt = (BLE_Event_Scan_Report *)param;

        if (rpt->Rpt_Data[7] == 0x02)
        {
            printf("128bit UUID = ");
            for (i = 9; i < 25; i++)
            {
                printf("%02x", rpt->Rpt_Data[i]);
            }
            printf("\r\nMajor ID = ");
            for (i = 25; i < 27; i++)
            {
                printf("%x", rpt->Rpt_Data[i]);
            }
            printf("\r\nMinor ID = ");
            for (i = 25; i < 27; i++)
            {
                printf("%x", rpt->Rpt_Data[i]);
            }
            printf("    RSSI:%d\n", rpt->Rpt_Rssi);
            printf("\r\n");
        }
        //User can filter data from the received data(rpt->data)
        // received power
    }
    break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
//        BLE_Event_ConnParam *connParam = (BLE_Event_ConnParam *)param;

//        connId = connParam->connId;
//        ble_state = STATE_BLE_CONNECTION;
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
//        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;
//        ble_state = STATE_BLE_STANDBY;

//        printf("Disconnected, ID:%d, Reason:0x%X\n",disconnParam->connId, disconnParam->disconnectReson);
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
//        BLE_Event_ConnUpdateParam *updateConnParam = (BLE_Event_ConnUpdateParam *)param;
//        printf("Connection updated, ID:%d, Status:%d, Interval:%d, Latency:%d, Timeout:%d\n", updateConnParam->connId,updateConnParam->status, updateConnParam->connInterval, updateConnParam->connLatency, updateConnParam->supervisionTimeout);
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
//        BLE_Event_Phy_Update_Param *phy = (BLE_Event_Phy_Update_Param *)param;
//        printf("STATUS: %d, TX PHY: %d, RX PHY: %d\n",phy->status,phy->tx_Phy, phy->rx_Phy);
    }
    break;

    case BLECMD_EVENT_STK_GEN_METHOD:
    {
//        BLE_Event_Stk_Gen_Method *passkey_method = (BLE_Event_Stk_Gen_Method *)param;
//        if(passkey_method->Stk_Gen_Method == PASSKEY_ENTRY)
//        {
//            //Start scanning user-entered passkey.
//        }
//        else if(passkey_method->Stk_Gen_Method == PASSKEY_DISPLAY)
//        {
//            //user can generate a 6-digit random code, and display it for pairing.
//        }
    }
    break;

    case BLECMD_EVENT_PASSKEY_CONFIRM:
    {
//        BLE_Event_PassKey_Confirm *event_param = (BLE_Event_PassKey_Confirm *)param;

//        //enter a scanned Passkey or use a randomly generaated passkey.
//        setBLE_Pairing_PassKey(event_param->connId, HOGP_PAIRING_KEY);
    }
    break;

    case BLECMD_EVENT_AUTH_STATUS:
    {
//        BLE_Event_Auth_Status *auth_result = (BLE_Event_Auth_Status *)param;
//        printf("AUTH Report, ID:%d , STATUS:%d\n", auth_result->connId, auth_result->status);
    }
    break;

    case BLECMD_EVENT_SCAN_COMPLETE:
    {

    }
    break;

    default:
        break;
    }
}

