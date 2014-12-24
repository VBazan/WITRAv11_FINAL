/// @file       cloud.h
/// @copyright  Copyright (c) 2012 Toradex AG
///             [Software License Agreement]
/// $Author: agp $
/// $Rev: 1179 $
/// $Date: 2012-12-06 06:08:53 +0100 (Do, 06 Dez 2012) $
/// @brief      Contains header files, function used AWS Cloud S3/Sns/Sqs Services
/// @target     Colibri Modules [PXAxxx, Tegra T20], WinCE 6.0 / 7.0
/// @caveats

#ifndef _CLOUD_H_
#define _CLOUD_H_

#include "awss3.h"
#include "awssns.h"
#include "awssqs.h"

//******************************************************************************
/// Gets Library Version
/// @param[in]     majorNumber pointer to major number
/// @param[in]     minorNumber pointer to minor number
/// @param[in]     buildNumber pointer to build number
/// @retval        nothing
void CloudGetLibVersion(DWORD *majorNumber, DWORD *minorNumber, DWORD *buildNumber);

//******************************************************************************
/// Gets local file size in bytes
/// @param[in]    filePath  pointer to the file path string
/// @param[out]   fileSize  pointer to the file size 
/// @retval       TRUE      Success
/// @retval       FALSE     See GetLastError()  
BOOL GetLocalFileSizeInBytes(char *filePath, DWORD *fileSize);
#endif //_CLOUD_H_

