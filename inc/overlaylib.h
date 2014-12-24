///
/// @file        OverlayLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 1819 $ 
/// $Date: 2013-09-23 13:46:32 +0200 (Mo, 23 Sep 2013) $
/// @brief       Library of support functions for Overlays.
/// 
/// @target      PXAxxx
/// @caveats     For T20 support see the Colibri Camera Library
/// 

#ifndef _OVERLAYLIB_H_
#define _OVERLAYLIB_H_

#include "MapRegLib.h"

#ifdef __cplusplus
extern "C" {
#endif

//Version Info
#define OVL_VER_MAJ     0
#define OVL_VER_MIN     2
#define OVL_VER_BUILD   1

//Overlay Structure
typedef struct
{
    unsigned long OverlayHeight;
    unsigned long OverlayWidth;
    unsigned long X_Position;
    unsigned long Y_Position;
    unsigned long Format;
    unsigned long DegradeBaseFrame;
    MEM_T CH2_Y;        //Overlay2
    MEM_T CH3_Cb;       //Overlay2
    MEM_T CH4_Cr;       //Overlay2
    unsigned long OverlayBPP;
    unsigned long TmpBPP;
    unsigned long ch2_size;
    unsigned long ch3_size;
    unsigned long ch4_size;
    unsigned long HWScaleY;        //16= NoScaling
    unsigned long DoubleBuf;
    MEM_T CH1;          //Overlay1
    unsigned long ch1_size;
    unsigned long ovNumber;
    unsigned long currentBuffer;
} XLLP_OVERLAY_T, *P_XLLP_OVERLAY_T;

// Use the following for configuring overlay 2 format
#define FORMAT_RGB           0x0
#define FORMAT_PACKED_444    0x1
#define FORMAT_PLANAR_444    0x2
#define FORMAT_PLANAR_422    0x3
#define FORMAT_PLANAR_420    0x4

// Use the following for base frame values
#define BPP_1             0x001
#define BPP_2             0x002
#define BPP_4             0x004
#define BPP_8             0x008
#define BPP_16            0x010
#define BPP_18            0x020
#define BPP_18_PACKED     0x040
#define BPP_19            0x080
#define BPP_19_PACKED     0x100
#define BPP_24            0x200
#define BPP_25            0x400

// Use the following for overlay values
#define O_BPP_4           0x2
#define O_BPP_8           0x3
#define O_BPP_16          0x4
#define O_BPP_18          0x5
#define O_BPP_18_PACKED   0x6
#define O_BPP_19          0x7
#define O_BPP_19_PACKED   0x8
#define O_BPP_24          0x9
#define O_BPP_25          0xA

//******************************************************************************
/// Get the library Version.
/// @param[out]         pVerMaj         Major version number.
/// @param[out]         pVerMin         Minor version number.
/// @param[out]         pBuild          Build number.
void OVLGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Initialise the library - call before using other functions
/// @retval             TRUE = Success
BOOL LCDInit(void);

//******************************************************************************
/// Release the library - call when finished with the library
/// @retval             TRUE = Success
void LCDDeInit(void);

//******************************************************************************
/// Degrades the dispay to 1 bit per pixel (to save bandwidth)
/// @param[in]          degrade         TRUE: Degrade, FALSE: Normal display                  
/// @param[in]          color           Palette color entry
void LCDDegrade(BOOL degrade, DWORD color);

//******************************************************************************
/// Setup the overlay
/// @param[in]          pXllpOverlay    Pointer to Overlay data structure                   
/// @param[out]         pXllpOverlay    Pointer to Overlay data structure
/// @retval             TRUE = Success
BOOL Overlay_Setup(P_XLLP_OVERLAY_T pXllpOverlay);

//******************************************************************************
/// Enable the overlay
/// @param[in]          pXllpOverlay    Pointer to Overlay data structure                   
/// @param[out]         pXllpOverlay    Pointer to Overlay data structure
/// @retval             TRUE = Success
BOOL Overlay_Enable(P_XLLP_OVERLAY_T pXllpOverlay);

//******************************************************************************
/// Disable the overlay
/// @param[in]          pOv             Pointer to Overlay data structure                   
/// @param[out]         pOv             Pointer to Overlay data structure
void Overlay_Disable(P_XLLP_OVERLAY_T pOv);

//******************************************************************************
/// Get the overlay buffer
/// @param[in]          pOv             Pointer to Overlay data structure                   
/// @param[out]         pOv             Pointer to Overlay data structure
void Overlay_GetBuffer(P_XLLP_OVERLAY_T pOv);

//******************************************************************************
/// Flip the overlay buffer
/// @param[in]          pOv             Pointer to Overlay data structure                   
/// @param[out]         pOv             Pointer to Overlay data structure
void Overlay_Flip(P_XLLP_OVERLAY_T pOv);

//******************************************************************************
/// Update the overlay Position, Size and Format
/// @param[in]          pOv             Pointer to Overlay data structure                  
/// @retval             TRUE = Success            
BOOL Overlay_PosSizeFormat(P_XLLP_OVERLAY_T pOv);

//******************************************************************************
/// Set the overlay transparency factor
/// @param[in]          n               Transparency level, 0: very transparent, 7: fully opaque                   
void SetTransparencyFactor(DWORD n);

//old version, do not use them anymore

//******************************************************************************
/// Setup the overlay - deprecated
/// @param[in]          pXllpOverlay    Pointer to Overlay data structure                   
/// @param[out]         pXllpOverlay    Pointer to Overlay data structure
/// @deprecated
BOOL Overlay2_Setup(P_XLLP_OVERLAY_T pXllpOverlay);

//******************************************************************************
/// Flip the overlay - deprecated
/// @param[in]          pOv             Pointer to Overlay data structure                   
/// @param[out]         pOv             Pointer to Overlay data structure
/// @deprecated
void Overlay2_Flip(P_XLLP_OVERLAY_T pOv);

//******************************************************************************
/// Get the overlay buffer - deprecated
/// @param[in]          pOv             Pointer to Overlay data structure                   
/// @param[out]         pOv             Pointer to Overlay data structure
/// @deprecated
void Overlay2_GetBuffer(P_XLLP_OVERLAY_T pOv);

//******************************************************************************
/// Disable the overlay - deprecated
/// @param[in]          pXllpOverlay    Pointer to Overlay data structure                   
/// @param[out]         pXllpOverlay    Pointer to Overlay data structure
/// @deprecated
void Overlay2_Disable(P_XLLP_OVERLAY_T pXllpOverlay);

//******************************************************************************
/// Enable the overlay - deprecated
/// @param[in]          pXllpOverlay    Pointer to Overlay data structure                   
/// @param[out]         pXllpOverlay    Pointer to Overlay data structure
/// @deprecated
void Overlay2_Enable(P_XLLP_OVERLAY_T pXllpOverlay);

#ifdef __cplusplus
}
#endif

#endif  //_OVERLAYLIB_H_
