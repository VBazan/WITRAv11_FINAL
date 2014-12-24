///
/// @file	     FlashLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: bis $
/// @version     $Rev: 1807 $ 
/// @date        $Date: 2013-09-10 14:45:58 +0200 (Di, 10 Sep 2013) $
/// @brief	     Library to manage interaction with Flash memory.
/// 
/// @target      PXAxxx, Tegra
/// @caveats     Not intended for public release, internal use only
/// 

#ifndef __FLASHLIB_H__
#define __FLASHLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

//Version Info
#define FLASH_VER_MAJ     2
#define FLASH_VER_MIN     4
#define FLASH_VER_BUILD   0

typedef enum _FLASHTYPE_
{
    FLASH_INVALID = 0,
	NOR_FLASH,
	NAND_FLASH,
	NAND_FLASH_TEGRA,
    EMMC_FLASH_TEGRA,
} FLASHTYPE;

#define FLASH_MUTEX     L"OnModuleFlash"

// Flash Error codes
#define FLASH_NO_ERROR					0
#define FLASH_GENERIC_ERROR				1
#define FLASH_BAD_BLOCK_DETECTED		2
#define FLASH_ECC_WARNING				3
#define FLASH_ECC_ERROR					4
#define FLASH_UNKNOWN_DEVICE			5
#define FLASH_INIT_FAILED				6
#define FLASH_RELOC_HEADER_INVALID		7
#define FLASH_ALL_RELOC_USED            8

#define NAND_UNKNOWN 0
#define NAND_K9F     1
#define NAND_K9K	 2
#define NAND_HY27    3
#define NAND_EN27    4

#define NOR_J3      1
#define NOR_K3      2
#define NOR_P30B    3
#define NOR_P30T    4
#define NOR_P33B    5
#define NOR_P33T    6

extern FLASHTYPE  g_FlashType;
extern DWORD      g_FlashModel;
extern DWORD      g_FlashSize;
extern DWORD      g_FlashLastError;

typedef BOOL     (*PFLASHERASE)(DWORD address, DWORD size);
typedef BOOL     (*PFLASHWRITE32)(DWORD address, DWORD value);
typedef BOOL     (*PFLASHWRITEBUFFER)(DWORD* pBuffer, DWORD address, DWORD length);
typedef DWORD    (*PFLASHREAD32)(DWORD address);
typedef BOOL     (*PFLASHREADBUFFER)(DWORD* pBuffer, DWORD address, DWORD length);
typedef DWORD    (*PFLASHGETBLOCKSIZE)(DWORD address);
typedef DWORD    (*PFLASHGETPAGESIZE)(DWORD address);
typedef BOOL     (*PFLASHUSEECC)(BOOL useEcc);

//******************************************************************************
/// Call this first to initialise the Flash Library
/// @retval		        Type of Flash present
FLASHTYPE FlashInit ();

//******************************************************************************
/// Call this when finished with the Flash Library
/// @retval		        Always returns TRUE
BOOL FlashDeInit();

//******************************************************************************
/// Waits on the Flash Mutex to be signalled
/// @param[in]	        timeout         Timeout in milliseconds
/// @retval		        TRUE            Mutex signalled
/// @retval		        FALSE           Timeout or Failure
BOOL FlashWaitMutex(DWORD timeout);

//******************************************************************************
/// Release the Flash Mutex    
/// @retval		        TRUE            Mutex Released
/// @retval		        FALSE           Error
BOOL FlashReleaseMutex();

#define FlashGetLastError() g_FlashLastError

typedef BOOL (*PPROGRESS_CALBACK_PROC)(DWORD done, DWORD size);

//******************************************************************************
/// Set the Flash Progress Callback Function and Interval
/// @param[in]	        pProc           Callback Function Pointer
/// @param[in]	        interval        Callback Interval
void FlashSetProgressCallback(PPROGRESS_CALBACK_PROC pProc, DWORD interval);

extern PFLASHERASE FlashErase;
extern PFLASHWRITE32 FlashWrite32;              
extern PFLASHWRITEBUFFER FlashWriteBuffer;
extern PFLASHREAD32 FlashRead32;
extern PFLASHREADBUFFER FlashReadBuffer;
extern PFLASHGETBLOCKSIZE FlashGetBlockSize;
extern PFLASHGETPAGESIZE FlashGetPageSize;
extern PFLASHUSEECC FlashUseECC;

/*************************************/
///    Colibri Tegra Functions
#define TEGRA_OPEN_READ    0x1  ///< Opens a file with read permissions.
#define TEGRA_OPEN_WRITE   0x2  ///< Opens a file with write persmissions.
#define TEGRA_OPEN_CREATE  0x4  ///< Creates a file if is not present on the file system.
#define TEGRA_OPEN_APPEND  0x8  ///< Open file in append mode. Implies WRITE.

//******************************************************************************
/// Open a Flash file - Tegra version
/// @param[in]	        FileName        Filename
/// @param[in]	        Flags           Open Flags
/// @param[in]          hFlash          File Instance Handle
/// @retval		        -1              Error
/// @retval             0               Success
/// @retval             Other           Error
DWORD TegraFlashOpen(char* FileName, DWORD Flags,  HANDLE * hFlash);

//******************************************************************************
/// Format flash file partition
/// @param[in]	        PartitionName   Name of the partition to be formated
/// @retval		        -1              Error
/// @retval             0               Success
/// @retval             Other           Error
DWORD TegraFlashFormat(char * PartitionName);

//******************************************************************************
/// Close a Flash file - Tegra version
/// @param[in]	        hFlash          Flash Instance Handle           
/// @retval		        -1              Error
/// @retval             0               Success
/// @retval             Other           Error
DWORD TegraFlashClose(HANDLE hFlash);

//******************************************************************************
/// Read from a Flash File - Tegra version
/// @param[in]	        hFlash          Flash Instance Handle
/// @param[in]	        BytesToRead     Number bytes to read
/// @param[in]	        outBuffer       Pointer to output buffer
/// @param[out]         BytesRead       Numbe rof bytes read
/// @retval		        -1              Error
/// @retval             0               Success
/// @retval             Other           Error
DWORD TegraFlashRead(HANDLE hFlash, void* outBuffer, DWORD BytesToRead, DWORD* BytesRead);

//******************************************************************************
/// Write to a Flash File - Tegra version
/// @param[in]	        hFlash          Flash Instance Handle
/// @param[in]	        BytesToWrite    Number  of bytes to write
/// @param[in]	        inBuffer        Pointer to input buffer
/// @param[out]         BytesWritten    Number of bytes written     
/// @retval		        -1              Error
/// @retval             0               Success
/// @retval             Other           Error
DWORD TegraFlashWrite(HANDLE hFlash, void* inBuffer, DWORD BytesToWrite, DWORD* BytesWritten);

//******************************************************************************
/// Seek to an offset in a Flash file - Tegra version
/// @param[in]	        hFlash          Flash Instance handle
/// @param[in]	        ByteOffset      Offset within the file
/// @param[in]          Whence          Seek from (0: Start, 1: Current Position, 2: End)   
/// @retval		        -1              Error
/// @retval             0               Success
/// @retval             Other           Error
DWORD TegraFlashSeek(HANDLE hFlash,  DWORD64 ByteOffset, DWORD Whence);

//******************************************************************************
/// Returns the library Version.
/// @param[out]         pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild          Returns the build number.         Set this parameter to NULL if not required.
///       
void FLASHGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif //__FLASHLIB_H__

