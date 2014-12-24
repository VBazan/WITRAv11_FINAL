///
/// @file	     CRCLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1138 $ 
/// @date        $Date: 2012-10-31 13:29:08 +0100 (Mi, 31 Okt 2012) $
/// @brief	     CRC calculations library
/// 
/// @target      PXAxxx, T20
/// 

#ifndef __CRCLIB_H__
#define __CRCLIB_H__

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRC_VER_MAJ     1
#define CRC_VER_MIN     0
#define CRC_VER_BUILD   0
    
/// Type and Version declarations

/// CRCH
#define CRC_MAGIC 0x48435243
#define CRC_VER   1

typedef struct
{
    DWORD      magic;
    WORD       version;
    WORD       len;         ///< length of the header
    DWORD      crc;
} CRCH;

typedef enum
{
    CRC_OK = 0,             ///< CRC check successful
    CRC_NOT_FOUND,          ///< CRC information not found
    CRC_WRONG_VERSION,      ///< Found CRC header but verion is not supported with this lib
    CRC_CALC_ERROR,         ///< Error during CRC calculation
    CRC_CRC_ERROR,          ///< Calculated CRC doesn't match saved CRC
} CRC_RET;

/// Defines for special handling
#define CRC_OPCODE  NULL
#define CRC_RESET   0
#define CRC_GET_CRC 1

/// Prototype for crc calculation functions
typedef BOOL (*PCRCCALCCRC)(void *pBuf, DWORD len, DWORD *pHash);

//******************************************************************************
/// Returns the library Version.
/// @param[out]         pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild          Returns the build number.         Set this parameter to NULL if not required.           
void CRCGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

/// Calculates Hash value based on different algorithms

/// The following functions have a special handling when called with pBuf==CRC_OPCODE (NULL)
/// The len parameter is the OP-Code in this case:
/// pBuf==CRC_OPCODE and len =  CRC_RESET:   RESET the code logic
/// pBuf==CRC_OPCODE and len =  CRC_GET_CRC: finalize the hash calculation and return the hash

//******************************************************************************
/// Calculates a hash value based on the WinCE bin CRC algorithm (simple addition).
/// @param[in]          *pBuf           Pointer to Input Buffer
/// @param[in]	        len             Input Buffer length
/// @param[out]         *pHash          Calculated binary Hash Value
/// @retval		        TRUE        
BOOL CRCCalcBinHash(BYTE *pBuf, DWORD len, DWORD *pHash);

//******************************************************************************
/// Calculates a hash value based on the Fletcher algorithm.
/// @param[in]          *pBuf           Pointer to Input Buffer
/// @param[in]	        len             Input Buffer length     
/// @param[out]         *pHash          Calculated Fletcher CRC 
/// @retval		        TRUE
/// @retval             FALSE
BOOL CRCCalcFletcherHash(DWORD *pBuf, DWORD len, DWORD *pHash);

// Other functions for CRC handling

//******************************************************************************
/// Find the CRC structure of an image.
/// @param[in]	        pImg             Start address of the image              
/// @param[in]	        allowFullSearch  Allow full search of the CRC header on a DWORD by DWORD base
/// @param[in]          maxSearchLen     Maximum search length in bytes if allowFullSearch is 1
/// @param[out]         pImgLen:         Return image len if CRC header was found
/// @param[out]             
/// @retval		        pCRC            Successful
/// @retval             NULL            Error
CRCH *CRCFindCRC(DWORD *pImg, DWORD *pImgLen, BOOL allowFullSearch, DWORD maxSearchLen);

//******************************************************************************
/// Checks CRC if present.
/// @param[in]          pImg             Start address of the image.
/// @param[in]	        allowFullSearch  Allow full search of the CRC header on a DWORD by DWORD base.
/// @param[in]	        maxSearchLen     Maximum search length in bytes if allowFullSearch is 1.
/// @param[out]         pCRC             Return CRCHeader if crc found.
/// @retval		        CRC_CALC_ERROR
/// @retval             CRC_OK
/// @retval             CRC_WRONG_VERSION
/// @retval             CRC_NOT_FOUND
CRC_RET CRCCheck(void *pImg, CRCH **pCRC, BOOL allowFullSearch, DWORD maxSearchLen);

#ifdef __cplusplus
}
#endif

#endif // __CRCLIB_H__