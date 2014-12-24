/// @file       splashscreen.h
/// @copyright  Copyright (c) 2012 Toradex AG
///             [Software License Agreement]
/// @copyright  Copyright (c) 2012 Toradex AG
/// @author	    $Author: lad $
/// @version    $Rev: 1459 $ 
/// @date       $Date: 2013-04-09 03:24:01 +0200 (Di, 09 Apr 2013) $
/// @brief      Contains function declarations used by splash screen library
/// @target     Colibri Modules [PXAxxx], WinCE 6.0 / 7.0
/// @caveats    It accepts only BMP files and color depth should be 8bbp or 256 Color\n
///             File Path string length should be less than 256. 

#ifndef _SPLASHSCREEN_H_
#define _SPLASHSCREEN_H_

// Header files
#include <windows.h>

#define FILE_PATH_STRING_LENGTH_MAX 	    256
#define COLOR_WIDTH_OFFSET                  0x1C

// Error Codes                              
#define ERROR_MODULE                        20
#define ERROR_INPUT_PARAMETER               21
#define ERROR_OPEN_INPUT_FILE               22
#define ERROR_CREATE_SS_DATA                23
#define ERROR_MEMORY_ALLOCATION             24
#define ERROR_READ_FILE                     25
#define ERROR_INVALID_FILE                  26
#define ERROR_FILE_NOT_8BPP                 27
#define ERROR_GENERATE_SS_DATA              28
#define ERROR_CONFIG_BLOCK_WRITE            29
#define ERROR_LOAD_FILE                     30
#define ERROR_SAVE_SS                       31
#define ERROR_FLASH_ADDRESS                 32



//******************************************************************************
/// Generates Splash Screen data from BMP file and saves it to the flash memory location.
/// @param[in]   bmpFilePath	File path of the BMP file. Maximum file path length should be less than 256.\n
///								File name should have an extension ".bmp" only.			
/// @param[in]   flashAddress	Flash address where file need to be stored. (Range of Flash address = )
/// @retval      FALSE          Success
/// @retval      ERROR CODE     See Macros Defined
/// @caveats     Function will return error\n
///              If file doesn't exists\n
///              File path string length is more than 256\n
///              File is not bmp\n
///              bmp file is not with 8bbp color depth\n
DWORD GenerateSplashScreen(WCHAR *bmpFilePath, WCHAR *flashAddress);
#endif //_SPLASHSCREEN_H_

