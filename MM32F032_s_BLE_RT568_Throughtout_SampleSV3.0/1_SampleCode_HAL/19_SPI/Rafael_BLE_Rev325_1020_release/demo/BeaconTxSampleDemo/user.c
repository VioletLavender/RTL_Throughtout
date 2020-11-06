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
#include "user.h"
#include "uart.h"



#ifdef _DEBUG_MSG_USER_
#include <stdio.h>
#endif  //(#ifdef _DEBUG_MSG_USER_)

uint8_t connId = 0;   //Current connection Id
Uint8 ble_state = STATE_BLE_STANDBY;

#define HOGP_PAIRING_KEY  654321                 //pairing key. uint32

/* Function prototype declaration */
static void BleEvent_Callback(BleCmdEvent event, void* param);


#ifdef _DEBUG_MSG_USER_
uint8_t gateTimeline = 0;
#endif  //(#ifdef _DEBUG_MSG_USER_)


#pragma push
//#pragma Otime
#pragma Ospace

//Advertising parameters
#define APP_ADV_INTERVAL_MIN       160U       //160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX       160U       //160*0.625ms=100ms

const uint8_t SET_ADV_DATA_IBEACON_FORMAT[] =
{
    30,                                                                           // iBecaon data format total Length
    GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, GAP_FLAGS_BR_EDR_NOT_SUPPORTED,      // GAP_ADTYPE_BREDR_NOT_SUPPORTED
    0x1A,                                                                         // length of this data including the data type byte
    GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,                                       // manufacturer specific data type
    0x4c,                                                                         // Company ID - Low Byte
    0x00,                                                                         // Company ID - High Byte
    0x02,                                                                         // Beacon
    0x15,                                                                         // data Length
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // 128 bit-UUID - Test
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01,                                                                   // Major ID
    0x00, 0x01,                                                                   // Minor ID
    0xC8,                                                                         // Power - The 2's complement of the calibrated Tx power
};


void BleApp_Init(void)
{
    /* register command event callback function */
    setBLE_RegisterBleEvent(BleEvent_Callback);
}

void Ble_Slave_AdvInit(void)
{
    BLE_Adv_Param advParam;

    advParam.Adv_Type = ADV_TYPE_ADV_NONCONN_IND;
    advParam.Adv_Interval_Min = APP_ADV_INTERVAL_MIN;
    advParam.Adv_Interval_Max = APP_ADV_INTERVAL_MAX;
    advParam.Adv_Channel_Map = ADV_CHANNEL_ALL;
    advParam.Adv_Filter_Policy = ADV_FILTER_POLICY_ACCEPT_ALL;

    setBLE_AdvParam(advParam);
}


void Ble_Slave_StartADV(void)
{
    Ble_Slave_AdvInit();
    setBLE_AdvData((uint8_t *)SET_ADV_DATA_IBEACON_FORMAT, sizeof(SET_ADV_DATA_IBEACON_FORMAT));             //SET_ADV_DATA can be modified by user. may depend on profile
    setBLE_AdvEnable();
}


//void Ble_Reset(void)
//{
//    /* reset BLE HW/SW */
//    //MCU re-start from main()
//    SYS_UnlockReg();
//    SYS_ResetChip();
//    SYS_LockReg();
//}

void trspx_receive_data_callback(uint8_t length, uint8_t *data)
{
#if (BLE_DEMO==DEMO_TRSPX_LED_SLAVE)   //User demo code example 1 - LED control
    //if(att_HDL_UDF01S_UDATRW01[0]==0x31)  {//user write '1'
    if (data[0] == 0x31)  //user write '1'
    {
        //turn on LED
        LED_1 = 0;
        //LED_2 = 0;
    }
    else
    {
        //turn off LED
        LED_1 = 1;
        //LED_2 = 1;
    }
#elif (BLE_DEMO==DEMO_TRSPX_UART_SLAVE) //User demo code example 2 - transparent transmission UART
    //show received RF data on UART
    UART_TX_Send(length, data);

#endif  //#if (BLE_DEMO==DEMO_TRSPX_LED_SLAVE)
}

void BleApp_Main(void)
{
#ifdef _DEBUG_MSG_USER_
    extern void msg2uart(Uint8 * CodeStr, Uint8 *HexStr, Uint8 HexStrLength);
#endif  //(#ifdef _DEBUG_MSG_USER_)

    if(ble_state == STATE_BLE_STANDBY)
    {
        Ble_Slave_StartADV();
        ble_state = STATE_BLE_ADVERTISING;
    }
    else if(ble_state == STATE_BLE_ADVERTISING)
    {

    }
} //end of BleApp_Main()
#pragma pop


/* GAP Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void* param)
{
    switch(event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
        if(ble_state == STATE_BLE_ADVERTISING)
        {
            printf("Advertising...\n");
        }
        break;

    case BLECMD_EVENT_SCAN_REPORT:
    {
//        BLE_Event_Scan_Report *rpt = (BLE_Event_Scan_Report *)param;

//        printf("RSSI:%d\n",rpt->Rpt_Rssi);
    }
    break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
        BLE_Event_ConnParam *connParam = (BLE_Event_ConnParam *)param;

        connId = connParam->connId;
        ble_state = STATE_BLE_CONNECTION;

        printf("Connected to %02x:%02x:%02x:%02x:%02x:%02x\n",connParam->peerAddr.addr[5],
               connParam->peerAddr.addr[4],
               connParam->peerAddr.addr[3],
               connParam->peerAddr.addr[2],
               connParam->peerAddr.addr[1],
               connParam->peerAddr.addr[0]);
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;
        ble_state = STATE_BLE_STANDBY;

        printf("Disconnected, ID:%d, Reason:0x%X\n",disconnParam->connId, disconnParam->disconnectReason);
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
        BLE_Event_ConnUpdateParam *updateConnParam = (BLE_Event_ConnUpdateParam *)param;
        printf("Connection updated, ID:%d, Status:%d, Interval:%d, Latency:%d, Timeout:%d\n", updateConnParam->connId,updateConnParam->status, updateConnParam->connInterval, updateConnParam->connLatency, updateConnParam->supervisionTimeout);
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
        BLE_Event_Phy_Update_Param *phy = (BLE_Event_Phy_Update_Param *)param;
        printf("STATUS: %d, TX PHY: %d, RX PHY: %d\n",phy->status,phy->phyParam.tx_Phy, phy->phyParam.rx_Phy);
    }
    break;

    case BLECMD_EVENT_STK_GEN_METHOD:
    {
        BLE_Event_Stk_Gen_Method *passkey_method = (BLE_Event_Stk_Gen_Method *)param;
        if(passkey_method->Stk_Gen_Method == PASSKEY_ENTRY)
        {
            //Start scanning user-entered passkey.
        }
        else if(passkey_method->Stk_Gen_Method == PASSKEY_DISPLAY)
        {
            //user can generate a 6-digit random code, and display it for pairing.
        }
    }
    break;

    case BLECMD_EVENT_PASSKEY_CONFIRM:
    {
        BLE_Event_PassKey_Confirm *event_param = (BLE_Event_PassKey_Confirm *)param;

        //enter a scanned Passkey or use a randomly generaated passkey.
        setBLE_Pairing_PassKey(event_param->connId, HOGP_PAIRING_KEY);
    }
    break;

    case BLECMD_EVENT_AUTH_STATUS:
    {
        BLE_Event_Auth_Status *auth_result = (BLE_Event_Auth_Status *)param;
        printf("AUTH Report, ID:%d , STATUS:%d\n", auth_result->connId, auth_result->status);
    }
    break;

    default:
        break;
    }
}

