///
/// @file        KernelCallbackLib
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author      $Author: gap $
/// @version     $Rev: 1708 $ 
/// @date        $Date: 2013-06-17 10:54:23 +0200 (Mo, 17 Jun 2013) $
/// @brief       Library to provide kernel routines calling
/// 
/// @target      PXAxxx, T20
/// 

#ifndef _KERNELCALLBACKLIB_H_
#define _KERNELCALLBACKLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#define KCB_VER_MAJ          1
#define KCB_VER_MIN          1
#define KCB_VER_BUILD        1


#define KCID_CALLBACK        0
#define KCID_GETKERNELFN     2

enum {
    idGetModuleHandle,
    idGetProcAddress,
    idAllocPhysMem,
    idCacheRangeFlush,
    idCacheSync, 
    idCeGetCacheInfo,
    idCeSetMemoryAttributes,
    idCeSetPowerOnEvent,
    idCeVirtualSharedAlloc,
    idCreateStaticMapping,
    idDrWatsonClear,
    idDrWatsonFlush,
    idDrWatsonGetSize,
    idDrWatsonReadData,
    idDrWatsonWriteData,
    idForcePageout,
    idFreeIntChainHandler,
    idFreePhysMem,
    idGetCallerProcess,
    idGetStdioPathW,
    idGetSystemMemoryDivision,
    idInterruptDisable,
    idInterruptDone,
    idInterruptInitialize,
    idInterruptMask,
    idKernelLibIoControl,
    idLoadDriver,
    idLoadIntChainHandler,
    idLockPages,
    idOpenWatchDogTimer,
    idPageOutModule, 
    idProfileCaptureStatus,
    idRefreshWatchDogTimer,
    idSetDbgZone,
    idSetInterruptEvent,
    idSetJITDebuggerPath,
    idSetStdioPathW, 
    idSetSystemMemoryDivision,
    idUnlockPages,
    idVirtualCopy,
    idVirtualCopyEx,
    idVirtualSetAttributes,
    idCloseHandle
};

extern void *KFn[];

//******************************************************************************
/// Initialises the kernel functions table.
/// @retval               TRUE
/// @retval               FALSE
BOOL GetKernelFn(void);

//******************************************************************************
/// Uses IOCTL for various utility function calls.
/// @param[in]            pArgs              
/// @param[in]            size            Size of arguments.         
/// @retval               TRUE            Callback function succeeded.
/// @retval               FALSE           Error.
DWORD DoKernelCallBack(void *pArgs, DWORD size);

//******************************************************************************
/// Returns the library Version.
/// @param[out]           pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]           pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]           pBuild          Returns the build number.         Set this parameter to NULL if not required.            
void KCBGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif // _KERNELLCALLBACKLIB_H_
