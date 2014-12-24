///
/// @file        MemLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: bis $
/// $Rev: 1848 $ 
/// $Date: 2013-10-14 10:50:52 +0200 (Mo, 14 Okt 2013) $
/// @brief       Library function to set memory timings for the Colibri PXA270 and PXA3xx.
/// 
/// @target      PXAxxx
/// 

#ifndef __COLIBRI_MEMLIB_H__
#define __COLIBRI_MEMLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "params.h"

// Library Version Info
#define MEM_VER_MAJ     1        ///< major library version
#define MEM_VER_MIN     0        ///< minor library version
#define MEM_VER_BUILD   2        ///< build revision

// Valid types for MSCCFG.type
#define MEMTYPE_FLASH   0
#define MEMTYPE_SRAM    1
#define MEMTYPE_BURST4  2
#define MEMTYPE_BURST8  3
#define MEMTYPE_VLIO    4

// Valid sizes for MSCCFG.buswidth
#define MEMBUS_32BIT    0
#define MEMBUS_16BIT    1

// Valid speeds for MSCCFG.buffer
#define MEMBUFFER_SLOW  0
#define MEMBUFFER_FAST  1

//-----------------------------------------------------------------------------
/// Set the memory timings for a particular chip select
/// @param[in]    dwBaseAddr    Base address of the memory region that should be configured.\n
///                             valid addresses are: (PXA270) 0x04000000 0x0C000000 0x10000000 \n
///                                                  (PXA3xx) 0x14000000 0x16000000 0x17000000
/// @param[in]    pTimings      requested timings. See params.h for details
/// @retval       TRUE          Success
/// @retval       FALSE         Failure
BOOL SetMemTimingsEx(DWORD dwBaseAddr, MSCCFG *pTimings);

//-----------------------------------------------------------------------------
/// Returns the library Version
/// @param[out]   pVerMaj       major version number.\n Set this parameter to NULL if not required.
/// @param[out]   pVerMin       minor version number.\n Set this parameter to NULL if not required.
/// @param[out]   pBuild        build number.\n Set this parameter to NULL if not required.
/// @retval       TRUE          Success
/// @retval       FALSE         Failure
void MEMGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif

