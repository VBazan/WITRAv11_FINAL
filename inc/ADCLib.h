///
/// @file        ADCLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 1819 $ 
/// $Date: 2013-09-23 13:46:32 +0200 (Mo, 23 Sep 2013) $
/// @brief       Library to support ADC interfacing 
/// 
/// @target      PXAxxx, T20
/// 

#ifndef _ADCINFOLIB_H_
#define _ADCINFOLIB_H_

#define CODEC_UNKNOWN   0
#define CODEC_UCB1400   1
#define CODEC_WM9713    2
#define CODEC_WM9715    3

#ifdef __cplusplus
extern "C" {
#endif

/// Version Info
#define ADC_VER_MAJ     2
#define ADC_VER_MIN     1
#define ADC_VER_BUILD   2

//******************************************************************************
/// Initializes the AC97 interface.
/// @retval             TRUE             Success.
/// @retval             FALSE            Error.
BOOL Ac97Init(void);

//******************************************************************************
/// Deinitializes the AC97 interface.
/// @retval             TRUE             Success.
/// @retval             FALSE            Error.
BOOL Ac97DeInit(void);

//******************************************************************************
/// Reads the library Version.
/// @param[out]         pVerMaj          Major version number.
/// @param[out]         pVerMin          Minor version number.
/// @param[out]         pBuild           Build number.
void ADCGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Reads which codec is installed on the current Colibri module.
/// @retval             0                Codec unknown.
/// @retval             1                UCB1400.
/// @retval             2                WM9713.
/// @retval             3                WM9715.
DWORD getCodecType(void); 

//******************************************************************************
/// Reads one ADC channel from Colibri's audio codec.
/// @param[in]          channel          ADC channel number (0..3)                   
/// @param[in]          samples          number of samples that should be taken, they will be summed up
/// @param[out]         value            raw ADC value, if sampels>1, value is the SUM of all measurements.
/// @retval             TRUE             Success.
/// @retval             FALSE            Error.
BOOL  ReadADCChannel(BYTE channel, DWORD* value, DWORD samples);

//******************************************************************************
/// Gets Converstion constant Microvolts per LSB. This can be different for each ADC and even for each channel.
/// @param[in]          channel          ADC channel number (0..3)
/// @retval             (7500000/1024)   Conversion constant Microvolts per ADC LSB.
/// @retval             (3300000*3/4096) Conversion constant Microvolts per ADC LSB.
/// @retval             (3300000  /4096) Conversion constant Microvolts per ADC LSB.
/// @retval             0                Error.
DWORD MicrovoltsPerLsb (BYTE channel);

//******************************************************************************
/// Reads one value from the touch controller
/// @param[in]          mode             X or Y axis measurement.
/// @param[in]          samples          Number of samples required.
/// @retval             value            Position value.
/// @retval             FALSE            No access.
/// @retval             -1               Error.
///
/// Remarks:            Only the UCB1400 codec is currently supported.
DWORD ReadTouch(BYTE mode, DWORD filter);

//******************************************************************************
/// Directly writes to an AC97 register
/// @param[in]          offset           Register address.
/// @param[in]          data             16-bit value to write into the register.
/// @retval             TRUE             Success.
/// @retval             FALSE            Error.
BOOL Ac97Write(UINT16 offset, UINT16 data); 

//******************************************************************************
/// Directly reads an AC97 register.
/// @param[in]          offset           Register address.
/// @param[out]         pdata            16-bit value read from the register.
/// @retval             TRUE             Success.
/// @retval             FALSE            Error.
BOOL Ac97Read(UINT16 offset, UINT16 *pdata);

#ifdef __cplusplus
}
#endif

#endif //_ADCINFOLIB_H_

