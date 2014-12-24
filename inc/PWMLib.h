///
/// @file        PWMLib.h
/// @copyright   Copyright (c) 2013 Toradex AG
/// $Author: gap $
/// $Rev: 1760 $ 
/// $Date: 2013-08-13 17:41:54 +0200 (Di, 13 Aug 2013) $
/// @brief       Library to support Pulse Width Modulation (PWM)
/// 
/// @test        Colibri Tegra2/PXA, [WinCE6] Image 0.1 PXA/Tegra on Iris
/// @target      PXAxxx, T20
/// 

#ifndef __PWMLIB_H__
#define __PWMLIB_H__

#include <windows.h> 

#ifdef __cplusplus
extern "C" {
#endif

///Version Info
#define PWM_VER_MAJ     2
#define PWM_VER_MIN     1
#define PWM_VER_BUILD   2

#define COLIBRI_PWM1    0
#define COLIBRI_PWM2    1
#define COLIBRI_PWM3    2
#define COLIBRI_PWM4    3

#define APALIS_PWM1     3
#define APALIS_PWM2     2
#define APALIS_PWM3     1
#define APALIS_PWM4     0

//******************************************************************************
/// Initialise PWM Controller (0-3)
/// @param[in] prescale:    scale factor for the 13Mhz Clock (1-64)
/// @param[in] period:      period length in scaled clocks units. Note - This is fixed for Tegra 2 i.e. 256
void InitPWM(DWORD pwm, DWORD prescale, DWORD period);

//******************************************************************************
/// Assign duty cycle in PWM Controller
/// @param[in] pwm:         channel to use (0-3)
/// @param[in] duty:        high time in scaled clocks units for each period i.e. 256
void SetPWMDuty(DWORD pwm, DWORD duty);

//******************************************************************************
/// Assign percentage duty cycle in PWM Controller
/// @param[in] pwm:         channel to use (0-3)
/// @param[in] duty:        high time in percantage (0-100)
void SetPWMDutyPercentage(DWORD pwm, DWORD duty);

//******************************************************************************
/// Deinitialise PWM Controller
/// @param[in] pwm:         channel to use (0-3)
void DeInitPWM(DWORD pwm);

//******************************************************************************
/// get the library version
/// @param[Out] pVerMaj     Returns the major version number. Set this parameter to NULL if not required.
/// @param[Out] pVerMin:    Returns the minor version number. Set this parameter to NULL if not required.
/// @param[Out] pBuild:     Returns the build number.         Set this parameter to NULL if not required.
void PWMGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif