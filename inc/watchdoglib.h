/// 
/// @file        WatchdogLib.h
/// @copyright   Copyright (c) 2010 Toradex AG
/// $Author: gap $
/// $Rev: 1760 $
/// $Date: 2013-08-13 17:41:54 +0200 (Di, 13 Aug 2013) $
/// @brief         WatchDog Support Library
///
/// @test        Colibri PXA320, Windows CE 6.0
/// @target      PXAxxx
/// @caveats     The watchdog does not work on PXA270 due to bug in the PXA270 CPU.
/// 
#ifndef __WDOG_H__
#define __WDOG_H__

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WATCHDOG_VER_MAJ    1    ///< Major Library Version
#define WATCHDOG_VER_MIN    0    ///< Library Subversion
#define WATCHDOG_VER_BUILD    0    ///< Library Build

//******************************************************************************
/// returns the version of this library
/// @param[out]    pVerMaj    Main Version of this library
/// @param[out]    pVerMin    Subversion of this library
/// @param[out]    pBuild     Build Number of this library
void WatchdogGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Activates the watchdog, and sets the Watchdog timer to the maximum timeout:\n
/// 2^32 ticks / 3.125MHz = 1'374sec = 23min.
/// @retval        TRUE = success
/// @retval        FALSE = error
BOOL WatchdogInit(void);

//******************************************************************************
/// Deactivate watchdog\n
/// This is *not* possible, therefore the function always returns FALSE
/// @retval        TRUE = success
/// @retval        FALSE = error
BOOL WatchdogDeinit(void);

//******************************************************************************
/// Retrigger Watchdog
/// @param[in] Timeout_ms A hardware reset will be done if the Watchdog is not\n
///                       retriggered within this time [ms] \n 
///                       valid range: 1...1'374'389 
/// @retval        TRUE = success
/// @retval        FALSE = error
BOOL WatchdogRetrigger(DWORD Timeout_ms);

#ifdef __cplusplus
}
#endif

#endif