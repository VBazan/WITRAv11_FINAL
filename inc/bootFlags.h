///
/// @file	     bootFlags.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1138 $ 
/// @date        $Date: 2012-10-31 13:29:08 +0100 (Mi, 31 Okt 2012) $
/// @brief	     Boot Flags & Defines
/// 
/// @target      PXAxxx, T20
/// 

#ifndef __BOOT_FLAGS_H__
#define __BOOT_FLAGS_H__

#ifdef __cplusplus
extern "C" {
#endif


/// Bootflag defines
//******************************************************************************
#define BF_FLASH_POS                   0x10     ///< The bootflags are located in the fourth dword of the eboot.

/// BootFlags Masks
#define BF_STATE_MASK                  0x0000000F
#define BF_FLAGS_MASK                  0x0000FFF0
#define BF_LAUNCHADDR_MASK             0xFFFF0000

/// BootFlags States
#define BF_BL                          (1<< 0)   ///< Normal Bootloader.
#define BF_FSBL_FIRST                  (1<< 1)   ///< First failSafe Bootloader.
#define BF_FSBL_FIRST_FAILSAFE         (1<< 2)   ///< First failSafe Bootloader if second FS bootloader failed.
#define BF_FSBL_SECOND                 (1<< 3)   ///< Second failSafeBootloader.

/// BootFlags Error Flags
#define BF_BL_ERROR                    (1<< 4)   ///< Normal Bootloader Error.
#define BF_FSBL_FIRST_ERROR            (1<< 5)   ///< First failSafe Bootloader Error.
#define BF_FSBL_SECOND_ERROR           (1<< 6)   ///< Second failSafe Bootloader Error.

/// BootFlags general Flags
#define BF_SHOW_MENU                   (1<< 7)   ///< Show the bootloader menu when starting.

/// Bootflags macros
//******************************************************************************
#define BF_SET_STATE(a, b)       ((a) = ((a) & ~BF_STATE_MASK)      | ((b)&BF_STATE_MASK))

#define BF_SET_LAUNCHADDR(a, b)  ((a) = ((a) & ~BF_LAUNCHADDR_MASK) | ((b)&BF_LAUNCHADDR_MASK))

#define BF_SET_FLAGS(a, b)       ((a) = ((a)                        | ((b)&BF_FLAGS_MASK)))

#ifdef __cplusplus
}
#endif

#endif //__BOOT_FLAGS_H__