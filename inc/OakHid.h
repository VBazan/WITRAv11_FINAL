///
/// @file	     OakHid.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1138 $ 
/// @date        $Date: 2012-10-31 13:29:08 +0100 (Mi, 31 Okt 2012) $
/// @brief	     Oak Family HID definitions
/// 
/// @target      Generic
/// 


#ifndef _OAKHID_H_
#define _OAKHID_H_

#include <windows.h>
#include "WinIOCTL.h"

#ifdef __cplusplus
extern "C" {
#endif

//	Toradex Global Vendor ID
#define TORADEX_VENDOR_ID	0x1B67

// Oak Sensor hot plugging Event Names 
#define OAK_ATTACH_EVENT	_T("HidOak-Attach")
#define OAK_DETACH_EVENT	_T("HidOak-Detach")
// Events contain data 


// *** DeviceIOControl parameters ***

// HID  Reports:
// Parameteres: lpOutBuf, nOutBuf, lpBytesRet
#define IOCTL_OAK_GET_INPUT_REPORT			CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF00, METHOD_BUFFERED, FILE_READ_ACCESS)
// Parameteres: lpInBuf, nInBuf
#define IOCTL_OAK_SET_OUTPUT_REPORT			CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF00, METHOD_BUFFERED, FILE_WRITE_ACCESS)

// HID Feature Reports:
// Parameteres: lpOutBuf, nOutBuf, lpBytesRet
#define IOCTL_OAK_GET_FEATURE_REPORT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF02, METHOD_BUFFERED, FILE_READ_ACCESS)
// Parameteres: lpInBuf, nInBuf
#define IOCTL_OAK_SET_FEATURE_REPORT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF02, METHOD_BUFFERED, FILE_WRITE_ACCESS)

// HID Report Length: nBytes [DWORD]
// Parameteres: lpOutBuf, nOutBuf, lpBytesRet
#define IOCTL_OAK_GET_INPUT_REPORT_LENGTH	CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF04, METHOD_BUFFERED, FILE_READ_ACCESS)
#define IOCTL_OAK_GET_OUTPUT_REPORT_LENGTH	CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF05, METHOD_BUFFERED, FILE_READ_ACCESS)
#define IOCTL_OAK_GET_FEATURE_REPORT_LENGTH	CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF06, METHOD_BUFFERED, FILE_READ_ACCESS)

// Number of Values in the HID Report: nValues [DWORD]
// Parameteres: lpBufOut, nOutBuf, lpBytesRet 
#define IOCTL_OAK_GET_INPUT_VALUE_COUNT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF08, METHOD_BUFFERED, FILE_READ_ACCESS)
#define IOCTL_OAK_GET_OUTPUT_VALUE_COUNT	CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF09, METHOD_BUFFERED, FILE_READ_ACCESS)

// HID Report Value Capability Array: HIDP_VALUE_CAPS[nValues], nValues = *lpBytesRet/sizeof(HIDP_VALUE_CAPS)
// Parameteres: lpOutBuf, nOutBuf, lpBytesRet
#define IOCTL_OAK_GET_INPUT_VALUE_CAPS		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF0C, METHOD_BUFFERED, FILE_READ_ACCESS)
#define IOCTL_OAK_GET_OUTPUT_VALUE_CAPS		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF0D, METHOD_BUFFERED, FILE_READ_ACCESS)

// Device Strings: [LPWSTR] 
// Parameteres: lpOutBuf, nOutBuf, lpBytesRet
#define IOCTL_OAK_GET_STRING_MANUFACT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF10, METHOD_BUFFERED, FILE_READ_ACCESS)
#define IOCTL_OAK_GET_STRING_PRODUCT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF11, METHOD_BUFFERED, FILE_READ_ACCESS)
#define IOCTL_OAK_GET_STRING_SERIAL			CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF12, METHOD_BUFFERED, FILE_READ_ACCESS)
// Additional Parameteres: *lpInBuf = StringIndex, nInBuf = sizeof(DWORD)
#define IOCTL_OAK_GET_STRING_INDEXED		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF13, METHOD_BUFFERED, FILE_READ_ACCESS)

// Queue Size: nBytes [DWORD]
// Parameteres: lpOutBuf, nBufOut, lpBytesRet
#define IOCTL_OAK_GET_QUEUE_SIZE			CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF20, METHOD_BUFFERED, FILE_READ_ACCESS)
// Parameteres: lpInBuf, nInBuf
#define IOCTL_OAK_SET_QUEUE_SIZE			CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF20, METHOD_BUFFERED, FILE_WRITE_ACCESS)

// Access Timeout: ms [DWORD]
// Parameteres: lpBufOut, nOutBuf, lpBytesRet 
#define IOCTL_OAK_GET_ACCESS_TIMEOUT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF21, METHOD_BUFFERED, FILE_READ_ACCESS)
// Parameteres: lpInBuf, nInBuf
#define IOCTL_OAK_SET_ACCESS_TIMEOUT		CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF21, METHOD_BUFFERED, FILE_WRITE_ACCESS)

// Device Info: PID [DWORD]
// Parameteres: lpOutBuf, nOutBuf, lpBytesRet
#define IOCTL_OAK_GET_PRODUCT_ID			CTL_CODE(FILE_DEVICE_UNKNOWN, 0xF24, METHOD_BUFFERED, FILE_READ_ACCESS)
// if PID = 0xFFFFFFFF, sensor has not been validated for this driver yet

#ifdef __cplusplus
}
#endif

#endif // _OAKHID_H_
