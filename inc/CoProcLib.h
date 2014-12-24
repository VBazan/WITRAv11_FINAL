///
/// @file        CoProcLib.h
/// @copyright   Copyright (c) 2013 Toradex AG
/// $Author: kia $
/// $Rev: 2135 $ 
/// $Date: 2014-04-01 09:28:39 +0200 (Di, 01 Apr 2014) $
/// @brief       Library to provide access to the ARM Coprocessor registers  
/// 
/// @target      Colibri PXAxxx, T20, T30 & Apalis T30
///

#ifndef __COPROC_LIB_H__
#define __COPROC_LIB_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CPL_VER_MAJ     2   ///< CoprocLib Major Library Version
#define CPL_VER_MIN     2   ///< CoprocLib Minor Library Version
#define CPL_VER_BUILD   0   ///< CoprocLib Library Build Version

// Defines for GetSoCType()
#define PXA 1
#define TEG 2

// Defines for getPROCID()
#define TYPE_PXA270     0x11
#define TYPE_MONAHANS   0x02
#define TYPE_PXA320     0x02
#define TYPE_PXA300XT   0x07
#define TYPE_PXA300     0x08
#define TYPE_PXA310     0x09
#define TYPE_TEGRA2     0x411FC09
#define TYPE_TEGRA3     0x412FC09

// Defines for getModuleFamily()
#define    MODULE_FAMILY_COLIBRI    0
#define    MODULE_FAMILY_APALIS     1


//-------------------------------------------------------------------------
/// Get Core Clock Configuration (Coprocessor 14, cr6)  [for PXA only] \r\n
/// On other CPUs the function also returns cp14.cr6, but this register has a different meaning.\r\n
/// See Marvell PXA3xx Developers Manual Vol. I for details.
/// @return     CLKCFG. (content of cp14.cr6)
DWORD getCLKCFG();

//-------------------------------------------------------------------------
/// Set Core Clock Configuration (Coprocessor 14, cr6)  [for PXA only] \r\n
/// On other CPUs the function also sets cp14.cr6, but this register has a different meaning.\r\n
/// See Marvell PXA3xx Developers Manual Vol. I for details.
/// @param[in]  CLKCFG. Value to store into cp14.cr6
void setCLKCFG(DWORD val);

//-------------------------------------------------------------------------
/// Get PWRMODE (Coprocessor 14, cr7)  [for PXA only] \r\n
/// On other CPUs the function also returns cp14.cr7, but this register has a different meaning.\r\n
/// See Marvell PXA3xx Developers Manual Vol. I for details.
/// @return     PWRMODE. (content of cp14.cr7)
DWORD getPWRMODE();

//-------------------------------------------------------------------------
/// Set PWRMODE (Coprocessor 14, cr7)  [for PXA only] \r\n
/// On other CPUs the function also sets cp14.cr7, but this register has a different meaning.\r\n
/// See Marvell PXA3xx Developers Manual Vol. I for details.
/// @param[in]  PWRMODE. Value to store into cp14.cr7
void setPWRMODE(DWORD val);

//-------------------------------------------------------------------------
/// Request extended CPUID (Coprocessor 15, register c0)
/// @return     value read from coprocessor 15 register c0.\r\n
///             Refer to the ARM documentation for details.
DWORD getCPUID();

//-------------------------------------------------------------------------
/// Detect the CPU type this code is running on
/// @return     CPU ID. See defines at the beginning of coproclib.h for details.
DWORD getPROCID();

//-------------------------------------------------------------------------
/// Request whether L2 cache is enabled or disabled
/// @retval true:   L2 cache is enabled 
/// @retval false:  L2 cache is disabled
DWORD getL2Enabled();

//-------------------------------------------------------------------------
/// Get the revision of this CoprocLib
/// @param[out]     major library version. Set this parameter to NULL if not required.
/// @param[out]     minor library version. Set this parameter to NULL if not required.
/// @param[out]     library build number.  Set this parameter to NULL if not required.
void CPLGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//-------------------------------------------------------------------------
/// Request SoC family type (PXA / Tegra)
/// @return CPU_TYPE:   SOC family i.e PXA or Tegra\r\n
///                     See defines at the beginning of coproclib.h for details.
DWORD GetSoCType();

//-------------------------------------------------------------------------
/// Request module family type (Colibri / Apalis)
/// @return CPU_TYPE:   MODULE_FAMILY_COLIBRI or MODULE_FAMILY_APALIS\r\n
///                     See defines at the beginning of coproclib.h for details.
DWORD getModuleFamily();

#ifdef __cplusplus
}
#endif

#endif
