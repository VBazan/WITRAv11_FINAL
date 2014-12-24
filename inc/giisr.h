///
/// @file	     GIISR.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1138 $ 
/// @date        $Date: 2012-10-31 13:29:08 +0100 (Mi, 31 Okt 2012) $
/// @brief	     Generic Installable Interrupt Service Routine definitions. Some changes to standard giisr.h
/// 
/// @target      PXAxxx, T20
/// 


#ifndef _GIISR_H_
#define _GIISR_H_

//#include <pkfuncs.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTL_CODE( DeviceType, Function, Method, Access ) (((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method))

#define IOCTL_KLIB_USER               256

#define USER_IOCTL(X) (IOCTL_KLIB_USER + (X))

#define IOCTL_GIISR_INFO USER_IOCTL(0)
#define IOCTL_GIISR_PORTVALUE USER_IOCTL(1)

#define MAX_GIISR_INSTANCES 32  // A maximum of 32 ISR handlers per dll are allowed

typedef struct _GIISR_INFO {
    DWORD SysIntr;              // SYSINTR for ISR handler to return (if associated device is asserting IRQ)
    BOOL CheckPort;             // If true, check port to see if device is asserting IRQ
    BOOL PortIsIO;              // Port is IO port (possibly true only for x86)
    BOOL UseMaskReg;            // If true, read from MaskAddr to obtain mask
    DWORD PortAddr;             // Port Address
    DWORD PortSize;             // Port data width in bytes
    DWORD Mask;                 // Mask to use on data port to determine if device is asserting IRQ
    DWORD MaskAddr;             // Address of register to use as mask
} GIISR_INFO, *PGIISR_INFO;

#ifdef __cplusplus
}
#endif

#endif // _GIISR_H_