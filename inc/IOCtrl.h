///
/// @file	     IOCtrl.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1682 $ 
/// @date        $Date: 2013-06-06 14:48:07 +0200 (Do, 06 Jun 2013) $
/// @brief	    
/// 
/// @target      
/// @caveats    
/// 

#ifndef _IOCTRL_H_
#define _IOCTRL_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	FLASH_SIZE = 0,
	FLASH_TYPE,
	FLASH_MODEL
} FLASHINFO;

typedef enum {
	MEMORY_SIZE = 0,
	MEMORY_TYPE,
	MEMORY_MODEL
} MEMORYINFO;

#define FILE_DEVICE_DISK            0x00000007
#define FILE_DEVICE_HAL				0x00000101
#define METHOD_BUFFERED				0
#define FILE_ANY_ACCESS				0
#define FILE_READ_ACCESS            1
#define FILE_WRITE_ACCESS           2

#define CTL_CODE( DeviceType, Function, Method, Access ) (((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method))

#define IOCTL_HAL_GET_UUID          CTL_CODE(FILE_DEVICE_HAL,  13,      METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_REBOOT			CTL_CODE(FILE_DEVICE_HAL,  15,      METHOD_BUFFERED, FILE_ANY_ACCESS)
	
#define IOCTL_PROCESSOR_INFORMATION	CTL_CODE(FILE_DEVICE_HAL,  25,		METHOD_BUFFERED, FILE_ANY_ACCESS)
                                                                        
#define IOCTL_HAL_REQUEST_SYSINTR	CTL_CODE(FILE_DEVICE_HAL,  38,      METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_RELEASE_SYSINTR	CTL_CODE(FILE_DEVICE_HAL,  54,      METHOD_BUFFERED, FILE_ANY_ACCESS)
                                                                        
#define IOCTL_HAL_GPIO2IRQ			CTL_CODE(FILE_DEVICE_HAL,  2048,    METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_IRQ2GPIO			CTL_CODE(FILE_DEVICE_HAL,  2049,    METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_IRQEDGE			CTL_CODE(FILE_DEVICE_HAL,  2050,    METHOD_BUFFERED, FILE_ANY_ACCESS)
                                                                        
#define IOCTL_HAL_GET_BSP_VER		CTL_CODE(FILE_DEVICE_HAL,  2051,    METHOD_BUFFERED, FILE_ANY_ACCESS)
                                                                        
#define IOCTL_HAL_CLEAR_REG			CTL_CODE(FILE_DEVICE_HAL,  2052,    METHOD_BUFFERED, FILE_ANY_ACCESS)
                                                                        
#define IOCTL_HAL_FLASHPARAMGET		CTL_CODE(FILE_DEVICE_HAL,  2058,    METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_FLASHPARAMSET		CTL_CODE(FILE_DEVICE_HAL,  2059,    METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_FLASHPARAMCLEAR	CTL_CODE(FILE_DEVICE_HAL,  2060,    METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_HAL_GETFLASHINFO		CTL_CODE(FILE_DEVICE_HAL,  2084,	METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_LOADIOCTLEXTENSION CTL_CODE(FILE_DEVICE_HAL, 2090, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_RELOADPMPARAMS     CTL_CODE(FILE_DEVICE_HAL, 2095, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_GETMEMORYINFO		CTL_CODE(FILE_DEVICE_HAL,  2122,	METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_GETBOOTFLAGS       CTL_CODE(FILE_DEVICE_HAL, 2123, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_HAL_ENABLENWE          CTL_CODE(FILE_DEVICE_HAL, 2124, METHOD_BUFFERED, FILE_ANY_ACCESS)
                                                                        
#define IOCTL_DISK_GET_SECTOR_ADDR  CTL_CODE(FILE_DEVICE_DISK, 0x714,   METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_DISK_READ             CTL_CODE(FILE_DEVICE_DISK, 0x702,   METHOD_BUFFERED, FILE_READ_ACCESS)
                                                                        
#define IOCTL_DISK_WRITE            CTL_CODE(FILE_DEVICE_DISK, 0x703,   METHOD_BUFFERED, FILE_WRITE_ACCESS)

#define IOCTL_DISK_FORMAT_VOLUME    CTL_CODE(FILE_DEVICE_DISK, 0x0088,  METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_DISK_SCAN_VOLUME      CTL_CODE(FILE_DEVICE_DISK, 0x0089,  METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_DISK_USER_START 0x7E0
#define IOCTL_DISK_USER(Function)   CTL_CODE( FILE_DEVICE_DISK, IOCTL_DISK_USER_START+Function, METHOD_BUFFERED, FILE_ANY_ACCESS)

// IOCTLs to accelerate FlashDisk init
#define IOCTL_DISK_SAVE_SMI         IOCTL_DISK_USER(13)


BOOL KernelIoControl(DWORD dwIoControlCode, LPVOID lpInBuf, DWORD nInBufSize, LPVOID lpOutBuf, DWORD nOutBufSize, LPDWORD lpBytesReturned);

typedef struct {
	DWORD id;
} FLASHPARAMGETPARAMS;
typedef struct {
	DWORD id;
	void* pBuf;
	DWORD len;
} FLASHPARAMSETPARAMS;


#ifdef __cplusplus
}
#endif

#endif //_IOCTRL_H_

