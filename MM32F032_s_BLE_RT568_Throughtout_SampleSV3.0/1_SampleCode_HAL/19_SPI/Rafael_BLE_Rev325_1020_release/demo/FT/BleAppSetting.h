#ifndef _BLEAPPSETTING_H_
#define _BLEAPPSETTING_H_

#include "mcu_definition.h"

/* ------------------------------------
 * Application Setting
  -------------------------------------*/
#define DEF_ENABLE_MSG  1

#if (DEF_ENABLE_MSG == 1)
#define D_msg           printf
#else
#define D_msg(...)
#endif


// Removed in SDK

//#define _HCI_ON_       //remove in SDK
#define _HCI_NEW_        //define in SDK
////#define _HCI_HW_       //remove in SDK. However, it is a must for test
//#define _HCI_HW_MSG_   //remove in SDK. (msg2uart)
//#define _LE_SCAN_ON_   //no use

#ifndef _HCI_NEW_
#undef _HCI_HW_
#endif
#ifndef _HCI_HW_
#undef _HCI_HW_MSG_      //remove in SDK
#endif

//#define _DEBUG_DISPLAY_      //remove in SDK
//#define _DEBUG_ICE_          //remove in SDK, no use
//#define _DEBUG_PINS_           //remove in SDK
//#define _DEBUG_MSG_USER_       //define in SDK
//#define _DEBUG_MSG_USER_SUB1 //remove in SDK


#ifdef _HCI_HW_
#undef _DEBUG_MSG_USER_
#endif  //(#ifdef _HCI_HW_)

#ifndef _DEBUG_MSG_USER_
#undef _DEBUG_MSG_USER_SUB1    //detailed debug message
#endif  //(#ifndef _DEBUG_MSG_USER_)

//#define _USED_EXCEPT_SDK_      //remove in SDK


// SDK: Company ID must to be changed to 0xFFFF
#define BLE_COMPANY_ID_L        0x64    //HCI__004
#define BLE_COMPANY_ID_H        0x08


#endif
