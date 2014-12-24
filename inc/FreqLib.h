///
/// @file        FreqLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 2101 $ 
/// $Date: 2014-03-10 14:27:38 +0100 (Mo, 10 Mrz 2014) $
/// @brief       Library to manage system clocks frequencies
/// 
/// @target      PXAxxx, Txx   
/// 

#ifndef __FREQLIB_H__
#define __FREQLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Params.h"

//Version Info
#define FRQ_VER_MAJ    2
#define FRQ_VER_MIN    1
#define FRQ_VER_BUILD  0

/// PXA Clocks Definitions
/////////////////////////////////////////
typedef enum
{
    PXACPUClockID,
    PXARunClockID,
    PXASysClockID,
    PXAMemClockID,
    PXASDRAMClockID,
    PXASRAMClockID,
    PXALCDClockID
} tPXAClockIDs;

typedef struct {
    DWORD cpuClk;
    DWORD runClk;
    DWORD sysClk;
    DWORD memClk;
    DWORD sdramClk;
    DWORD sramClk;
    DWORD lcdClk;
} FRQPXAClocks;

/// Tegra Clocks Definitions
/////////////////////////////////////////
typedef enum
{
    TEGCPUClockID,
    TEGSysClockID,
    TEGAVPClockID,
    TEGAHBClockID,
    TEGAPBClockID,
    TEGVIDClockID,
    TEGEMCClockID,
    TEGClockIDMax
} tTEGClockIDs;

typedef struct {
    DWORD cpuClk;
    DWORD sysClk;
    DWORD avpClk;
    DWORD ahbClk;
    DWORD apbClk;
    DWORD vidClk;
    DWORD emcClk;
} FRQTEGClocks;

#define TEG_DFS_STATE_STOPPED     2
#define TEG_DFS_STATE_CLOSED_LOOP 3

//******************************************************************************
/// Description:        Returns the library Version.
/// @param[out]         pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild          Returns the build number.         Set this parameter to NULL if not required.          
void FRQGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Description:        Initialises the frequency library.
/// @retval             TRUE            Success
/// @retval             FALSE           Failure          
BOOL FRQInit();

//******************************************************************************
/// Description:        De-initialises the frequency library.
void FRQDeInit();

//******************************************************************************
/// Description:        Returns the current PXA Clocks Frequencies.
/// @param[in/out]      pxaClocks       Pointer to FRQPXAClocks structure that will be updated 
/// @retval             TRUE            Success
/// @retval             FALSE           Failure          
BOOL FRQGetPXAClocks(FRQPXAClocks *pxaClocks);

//******************************************************************************
/// Description:        Returns the current Tegra Clocks Frequencies.
/// @param[in/out]      tegClocks       Pointer to FRQTEGClocks structure that will be updated 
/// @retval             TRUE            Success
/// @retval             FALSE           Failure          
BOOL FRQGetTEGClocks(FRQTEGClocks *tegClocks);

//******************************************************************************
/// Description:        Returns the current state of DFS (Dynamic Frequency Switching).
/// @retval             2               DFS Stopped
/// @retval             3               DFS Closed Loop
DWORD FRQGetTEGDFSState();

//******************************************************************************
/// Description:        Sets the PXA Multiplier Configuration.
/// @param[in]          multiplierConfig Multiplier Configuration Structure 
/// @retval             TRUE            Success
/// @retval             FALSE           Failure 
BOOL FRQSetPXAFrequencies(MULTCFG multiplierConfig);

//******************************************************************************
/// Set core voltage.
/// @param[in]          core_voltage    Voltage required in mV            
/// @retval             Voltage set in mV
/// Remarks:            !!!this is working on PXA270 only!!!
DWORD FRQSetPXA270Voltage(DWORD core_voltage);

//******************************************************************************
/// Description:        Sets the given Tegra clock to the given frequency.
/// @param[in]          tegClockID      ID of the Tegra Clock to be set 
/// @param[in]          frequency       Frequency to set
/// @retval             TRUE            Success
/// @retval             FALSE           Failure 
BOOL FRQSetTEGFrequency(tTEGClockIDs tegClockID, DWORD frequency);

//******************************************************************************
/// Description:        Sets the state of DFS (Dynamic Frequency Switching).
/// @param[in]          state           New DFS State (2 = Stopped, 3 = Closed Loop)
/// @retval             TRUE            Success
/// @retval             FALSE           Failure 
BOOL FRQSetTEGDFSState(DWORD state);

/// Deprecated functions

#define CLKCFG_T    1
#define CLKCFG_F    2
#define CLKCFG_HT   4
#define CLKCFG_B    8

#define MDREFR_APD      (1<<20)

#define MDREFR_K0DB2    (1<<14)
#define MDREFR_K1DB2    (1<<17)
#define MDREFR_K0DB4    (1<<29)

//******************************************************************************
/// Set the processor frequencies according to mp
/// @param[in]          mp                 Multiplier Configuration Structure
void SetFreq(MULTCFG mp);

//******************************************************************************
/// Get the actual processor frequencies  
/// @retval             Multiplier Configuration Structure
MULTCFG GetFreq(void);

//******************************************************************************
/// Calculate the different clocks in HZ
/// @param[in]          ms                 Multiplier Configuration Structure
/// @param[out]         pCPUClk            CPU Clock Speed (RunMode Clock * XN)
/// @param[out]         pRunClk            RunMode Clock Speed
/// @param[out]         pSysClk            System Clock Speed
/// @param[out]         pMemClk            Memory Clock Speed
/// @param[out]         pSdramClk          SDRAM Clock Speed
/// @param[out]         pSRamClk           SRAM Clock Speed
/// @param[out]         pLcdClk            LCD Clock Speed
void CalcSystemFreq(MULTCFG ms, DWORD* pCpuClk, DWORD* pRunClk, DWORD* pSysClk, DWORD* pMemClk, DWORD* pSdramClk, DWORD* pSramClk, DWORD* pLcdClk);

//******************************************************************************
/// Set core voltage.
/// @param[in]          core_voltage       Voltage in mV            
/// @retval             Set voltage in mV
/// Remarks:            !!!this is working on PXA270 only!!!
DWORD SetVoltage(DWORD core_voltage);

#ifdef __cplusplus
}
#endif

#endif //__FREQLIB_H__