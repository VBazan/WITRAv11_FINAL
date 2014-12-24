///
/// @file        SysInfoLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 2101 $ 
/// $Date: 2014-03-10 14:27:38 +0100 (Mo, 10 Mrz 2014) $
/// @brief       Library to provide access to System Information
/// @target      PXAxxx, T20
/// 

/// See also SysInfoLibDemo

#ifndef _SYSINFOLIB_H_
#define _SYSINFOLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SYSINFOLIBBASIC
#include "Params.h"
#include "ImgInfoLib.h"
#endif

#define SIL_VER_MAJ     1
#define SIL_VER_MIN     4
#define SIL_VER_BUILD   0

// Flash Type IDs
#define TYPE_NOR        0x01
#define TYPE_NAND       0x02

#ifdef SYSINFOLIBBASIC
// enable in public release
// BSP Version Structure 
typedef struct 
{
    DWORD ID;
    DWORD Maj;
    DWORD Min;
} BSPVER;

// HW Version Structure 
typedef struct
{
    WORD VerMaj;
    WORD VerMin;
    WORD Variant;
    WORD ProdID;
} HWCFG;

// User Data Structure (256 Bytes fix) (Advanced Lib only)
typedef union
{
    BYTE    bUserData[256];
    WORD    wUserData[128];
    DWORD   dwUserData[64];
} USERCFG;
#endif

// Processor Info Structure
typedef struct _PROCESSOR_INFO
{
    WORD    wVersion;
    WCHAR   szProcessCore[40];
    WORD    wCoreRevision;
    WCHAR   szProcessorName[40];
    WORD    wProcessorRevision;
    WCHAR   szCatalogNumber[100];
    WCHAR   szVendor[100];
    DWORD   dwInstructionSet;
    DWORD   dwClockSpeed;
} PROCESSOR_INFO;

#ifndef SYSINFOLIBBASIC
typedef struct 
{
   DWORD  From;
   DWORD  To;
} BB_RELOC;
#endif 

//******************************************************************************
/// Initialise the library - call this before any other functions in tis library
/// @retval             TRUE    SysInfoLib is initialized
/// @retval             FALSE   SysInfoLib is de-initialized
BOOL SysInfoLibInit(void);

// call the deinit if you don't need other functions of this lib anymore
//******************************************************************************
/// Frees the library - call this if you do not need the lbrary anymore.
/// @retval             TRUE    SysInfoLib is initialized
/// @retval             FALSE   SysInfoLib is de-initialized
BOOL SysInfoLibDeInit(void);

//******************************************************************************
/// get the Operating System version
/// @param[out]         pOSInfo            Pointer to OS Version Info Structure
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL  GetOSVersion(LPOSVERSIONINFO pOSInfo);

//******************************************************************************
/// Check the BSP version and date are valid 
/// @param[in]          pBspVer            Pointer to BSP version data structure
/// @param[in]          pBspDate           Pointer to the BSP release date 
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetImageInfo(BSPVER *pBspVer, SYSTEMTIME *pBspDate);

//******************************************************************************
/// Updates the BSP Release Date
/// @param[in]          pBspVer            pointer to BSP version data structure
/// @param[IN/OUT]      pBspDate           pointer to BSP release date structure
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetBootloaderInfo(BSPVER *pBspVer, SYSTEMTIME *pBspDate);

//******************************************************************************
/// Get the CPU, RAM and Bus speeds
/// @param[out]         pCpuSpeed          Pointer to CPU Speed [Hz] for PXA, [MHz] for Tegra
/// @param[out]         pRamSpeed          Pointer to RAM Speed [Hz] for PXA, [MHz] for Tegra
/// @param[out]         pBusSpeed          Pointer to Bus Speed [Hz] for PXA, [MHz] for Tegra
/// @retval             TRUE    Success. This function always returns TRUE
BOOL GetSystemSpeeds(DWORD *pCpuSpeed, DWORD *pRamSpeed, DWORD *pBusSpeed);

//******************************************************************************
/// Get the Processor information - see PROCESSOR_INFO definition
/// @param[out]         pPI                Pointer to Processor Info data structure
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetProcessorInfo(PROCESSOR_INFO *pPI);

//******************************************************************************
/// Get the board Version and Product ID 
/// @param[out]         pHWVer             Pointer to Board Version data structure
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetBoardVersion(HWCFG *pHWVer);

//******************************************************************************
/// Get the board Serial Number
/// @param[out]         pSerial            Pointer to Serial Number data 
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetBoardSerial(DWORD *pSerial);

//******************************************************************************
/// Get the MAC Address of the board
/// @param[out]         pMAC               Pointer to MAC Address data
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetMACAddress(BYTE *pMAC);

//******************************************************************************
/// Set the MAC Address of the board
/// @param[in]          pMAC               Pointer to MAC Address data
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL SetMACAddress(BYTE *pMAC);

//******************************************************************************
/// Get the size of the Flash
/// @param[out]         pFlashType         Pointer to Flash Type data
/// @return             Size of the flash memory [in Bytes]
DWORD GetFlashSize(DWORD* pFlashType);

//******************************************************************************
/// Get the RAM memory size
/// @param[out]         pFlashType         Pointer to Flash Type data
/// @return             Size of the RAM [in Bytes]
DWORD GetMemorySize(void);

#ifndef SYSINFOLIBBASIC
// This is not available in the free SysInfoLibBasic version. 
// If you want to use this functionality please contact Toradex.
//******************************************************************************
/// Get the bad block table
/// @param[out]         reloc              Pointer to Bad Block array 
/// @param[out]         numReloc           Number of Bad Block records
/// @retval             TRUE    Success
/// @retval             FALSE   Failure
BOOL GetBBTable(BB_RELOC *reloc, DWORD *numReloc);
#endif

//******************************************************************************
/// Set System Frequencies on PXA270
/// @param[in]          l                  Run Mode to 13 Mhz processor Oscillator Ratio
/// @param[in]          n                  Turbo Mode to Run Mode Ratio
/// @param[in]          turbo              Turbo Mode Enable
/// @param[in]          turbo2             Half-Turbo Mode Enable
/// @param[in]          fastbus            Fast Bus Mode Enable
/// @param[in]          altmem             Memory Controller Clock Alternate Setting Enable
/// @param[in]          sram2              SDRAM Clock Pin 1 Selector
/// @retval             0   No Error
/// @retval             1   l<0 | l>30
/// @retval             2   n<0 | n>6
/// @remarks            See also Colibri Tweak -> System Clocks
DWORD SetFrequencyPXA270(DWORD l, DWORD n, BOOL turbo, BOOL turbo2, BOOL fastBus, BOOL altmem, BOOL sram2);

//******************************************************************************
/// Set System Frequencies on PXA3xx
/// @param[in]          l                  Core PLL Run-Mode-to-Oscillator Ratio
/// @param[in]          n                  Turbo Mode to Run Mode Ratio
/// @param[in]          hsio               System Bus-Clock frequency Select
/// @param[in]          sram               SRAM Fequency Select
/// @param[in]          staticMem          Static Memory Controller Frequency Select
/// @retval             0    No Error
/// @retval             1    l<8 | l>31
/// @retval             2    n<1 | n>2
/// @retval             3    hsio not matched
/// @retval             4    sram not matched
/// @retval             5    staticMem not matched
/// @remarks            See also Colibri Tweak -> System Clocks
DWORD SetFrequencyPXA3xx(DWORD l, DWORD n, DWORD hsio, DWORD sram, DWORD staticMem);

// Set core voltage in [mV]
// Return core voltage [mV]
//******************************************************************************
/// Set core voltage
/// @param[in]          core_voltage        New core voltage [in mV]
/// @return             Voltage set [in mV]
/// @remarks            See also Colibri Tweak -> System Clocks
DWORD SetCoreVoltage(DWORD core_voltage);

#ifndef SYSINFOLIBBASIC
// This is not available in the free SysInfoLibBasic version. 
// If you want to use this functionality please contact Toradex.
//******************************************************************************
/// Get User Config Data
/// @param[out]         pUser               Pointer to User Config Data Structure
/// @return             Number of bytes read from flash. User Data is always 256 bytes.
DWORD GetUserData(USERCFG* pUser);

//******************************************************************************
/// Set User Config data
/// @param[in]          pUser               Pointer to User Config Data Structure
/// @return             Number of bytes written to flash. User Data is always 256 bytes. 0 = failure
DWORD SaveUserData(USERCFG* pUser);
#endif

//******************************************************************************
/// Returns the library Version.
/// @param[out]         pVerMaj             Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin             Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild              Returns the build number.         Set this parameter to NULL if not required.     
void SILGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif //_SYSINFOLIB_H_

