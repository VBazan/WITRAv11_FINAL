///
/// @file	     CPLDLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1138 $ 
/// @date        $Date: 2012-10-31 13:29:08 +0100 (Mi, 31 Okt 2012) $
/// @brief	     This library allows you to update the CPLD present on Colibri PXA3xx modules.
/// 
/// @target      PXA3xx
/// @caveats     While updating the CPLD the SODIMM pins 88, 90 and 92 have
///              to be left open (floating) because these pins were used to\n
///              program the CPLD. If that's not possible in your HW
///              you could use the Toradex Colibri EvalBoard for updating\n
///              the CPLD.
/// 

#ifndef _CPLDLIB_H_
#define _CPLDLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CUL_VER_MAJ     1
#define CUL_VER_MIN     3
#define CUL_VER_BUILD   0

#include <windows.h>

/// CPLDLib type definitions
typedef enum
{
	CUL_SUCCESS = 0,                    ///< No Error, Function returned sucessfully
	CUL_ERROR,                          ///< General Error
    CUL_NOT_INITIALIZED,                ///< The update lib (CUL) is not initialized.
	CUL_HW_NOT_SUPPORTED,               ///< The current HW (module) is not supported
    CUL_CPLD_NOT_FOUND,                 ///< No CPLD found
    CUL_CPLD_NOT_SUPPORTED,             ///< The CPLD on this module isn't supported or doesn't support a feature.
    CUL_CLD_READ_ERROR,                 ///< Could not read data from CPLD
    CUL_CLD_WRITE_ERROR,                ///< Could not write data to CPLD
} CUL_RET_TYPE;

//******************************************************************************
/// Returns the library Version
/// @param[out]     pVerMaj            Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]     pVerMin            Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]     pBuild             Returns the build number.         Set this parameter to NULL if not required.
void CULGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Initialize the CPLDLib.
/// @retval         CUL_SUCCESS
/// @retval         CUL_HW_NOT_SUPPORTED
/// @retval         CUL_CPLD_NOT_SUPPORTED
/// @retval         CUL_CPLD_NOT_FOUND
///
/// Remarks:        CULInit has to be called once before any of the following functions:
///                 - CULGetUserCode
///                 - CULDoUpdate
CUL_RET_TYPE CULInit(void);

//******************************************************************************
/// DeInitialize the CPLDLib
/// @retval         CUL_SUCCESS
///
/// Remarks:        The following functions don't work correctly after calling CULDeInit:
///                 - CULGetUserCode
///                 - CULDoUpdate
CUL_RET_TYPE CULDeInit(void);

//******************************************************************************
/// Reads the UserCode out of the CPLD if supported.
/// @param[out]     pUserCode         Returns the UserCode of the CPLD.
/// @retval         CUL_SUCCESS
/// @retval         CUL_NOT_INITIALIZED
/// @retval         CUL_CLD_READ_ERROR
CUL_RET_TYPE CULGetUserCode(DWORD *pUserCode);

//******************************************************************************
/// Writes the content of the buffer to the CPLD.
/// @param[in]	    pXSVFData        Pointer to a buffer containing the XSVF data to program the CPLD.
/// @param[in]	    mode             1:  Perform whole update with priority 1; Don't perform OnTheFly pattern activation.
/// @retval         CUL_SUCCESS
/// @retval         CUL_NOT_INITIALIZED
/// @retval         CUL_CLD_WRITE_ERROR
CUL_RET_TYPE CULDoUpdate(BYTE *pXSVFData, DWORD mode);

#ifdef __cplusplus
}
#endif

#endif // _CPLDLIB_H_

