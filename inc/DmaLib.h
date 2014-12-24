///
/// @file        DmaLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 1819 $ 
/// $Date: 2013-09-23 13:46:32 +0200 (Mo, 23 Sep 2013) $
/// @brief       Library for DMA handling, including channel allocation and DMA transfer control.
/// 
/// @test        All Colibri PXA modules, WinCE5 and WinCE6
/// @target      PXAxxx
/// @caveats     No support for T20 at the moment
/// 

#ifndef __DMALIB_H__
#define __DMALIB_H__

#include "MapRegLib.h"

#ifdef __cplusplus
extern "C" {
#endif

//Version Info
#define DMA_VER_MAJ     1
#define DMA_VER_MIN     3
#define DMA_VER_BUILD   1

#define DMA_MAX_CHAN    32        ///< number of available DMA channels
#define DMA_USAGE_LEN   32        ///< maximum string length for a descriptive text (DMA channel usage)

/// Memory mapped filename used by the DMALib for interprocess sharing of DMA data structures.
#define DMA_SHARED_FILENAME   (TEXT("ColibriDmaGlobals"))

/// Memory mapped DMA Globals structure - sorted by DMA channel
typedef struct
{
    CHAR    szDMAChanUsage[DMA_MAX_CHAN][DMA_USAGE_LEN];    ///< descriptive text on what the DMA channel is used for
    DWORD   dwDMAChanUsageID[DMA_MAX_CHAN];                 ///< ID of the device which uses this DMA channel. @n -1=free (see DMA_PERIPH_ID)
} DMA_GLOBALS;

/// DMA peripheral device DRCMR IDs
typedef enum
{
    DMA_PERIPH_DREQ0 = 0,
    DMA_PERIPH_DREQ1 = 1,
    DMA_PERIPH_SSP_4_RX = 2,
    DMA_PERIPH_SSP_4_TX = 3,
    DMA_PERIPH_BTUART_RX = 4,
    DMA_PERIPH_BTUART_TX = 5,
    DMA_PERIPH_FFUART_RX = 6,
    DMA_PERIPH_FFUART_TX = 7,
    DMA_PERIPH_AC97_MIC = 8,
    DMA_PERIPH_AC97_MODEM_RX = 9,
    DMA_PERIPH_AC97_MODEM_TX = 10,
    DMA_PERIPH_AC97_AUDIO_RX = 11,
    DMA_PERIPH_AC97_AUDIO_TX = 12,
    DMA_PERIPH_SSP_1_RX = 13,
    DMA_PERIPH_SSP_1_TX = 14,
    DMA_PERIPH_SSP_2_RX = 15,
    DMA_PERIPH_SSP_2_TX = 16,
    DMA_PERIPH_STUART_RX = 19,
    DMA_PERIPH_STUART_TX = 20,
    DMA_PERIPH_MMC_RX = 21,
    DMA_PERIPH_MMC_TX = 22,
    DMA_PERIPH_USB_ENDPOINT_0 = 24,
    DMA_PERIPH_USB_ENDPOINT_A = 25,
    DMA_PERIPH_USB_ENDPOINT_B = 26,
    DMA_PERIPH_USB_ENDPOINT_C = 27,
    DMA_PERIPH_USB_ENDPOINT_D = 28,
    DMA_PERIPH_USB_ENDPOINT_E = 29,
    DMA_PERIPH_USB_ENDPOINT_F = 30,
    DMA_PERIPH_USB_ENDPOINT_G = 31,
    DMA_PERIPH_USB_ENDPOINT_H = 32,
    DMA_PERIPH_USB_ENDPOINT_I = 33,
    DMA_PERIPH_USB_ENDPOINT_J = 34,
    DMA_PERIPH_USB_ENDPOINT_K = 35,
    DMA_PERIPH_USB_ENDPOINT_L = 36,
    DMA_PERIPH_USB_ENDPOINT_M = 37,
    DMA_PERIPH_USB_ENDPOINT_N = 38,
    DMA_PERIPH_USB_ENDPOINT_P = 39,
    DMA_PERIPH_USB_ENDPOINT_Q = 40,
    DMA_PERIPH_USB_ENDPOINT_R = 41,
    DMA_PERIPH_USB_ENDPOINT_S = 42,
    DMA_PERIPH_USB_ENDPOINT_T = 43,
    DMA_PERIPH_USB_ENDPOINT_U = 44,
    DMA_PERIPH_USB_ENDPOINT_V = 45,
    DMA_PERIPH_USB_ENDPOINT_W = 46,
    DMA_PERIPH_USB_ENDPOINT_X = 47,
    DMA_PERIPH_BASEBAND_1_RX = 48,
    DMA_PERIPH_BASEBAND_1_TX = 49,
    DMA_PERIPH_BASEBAND_2_RX = 50,
    DMA_PERIPH_BASEBAND_2_TX = 51,
    DMA_PERIPH_BASEBAND_3_RX = 52,
    DMA_PERIPH_BASEBAND_3_TX = 53,
    DMA_PERIPH_BASEBAND_4_RX = 54,
    DMA_PERIPH_BASEBAND_4_TX = 55,
    DMA_PERIPH_BASEBAND_5_RX = 56,
    DMA_PERIPH_BASEBAND_5_TX = 57,
    DMA_PERIPH_BASEBAND_6_RX = 58,
    DMA_PERIPH_BASEBAND_6_TX = 59,
    DMA_PERIPH_BASEBAND_7_RX = 60,
    DMA_PERIPH_BASEBAND_7_TX = 61,
    DMA_PERIPH_USIM_RX = 62,
    DMA_PERIPH_USIM_TX = 63,
    DMA_PERIPH_SSP_3_RX = 66,
    DMA_PERIPH_SSP_3_TX = 67,
    DMA_PERIPH_TPM_RX = 71,
    DMA_PERIPH_TPM_TX_1 = 72,
    DMA_PERIPH_TPM_TX_2 = 73,
    DMA_PERIPH_DREQ2 = 74,
    DMA_PERIPH_USIM_2_RX = 91,
    DMA_PERIPH_USIM_2_TX = 92,
    DMA_PERIPH_MMC_2_RX = 93,
    DMA_PERIPH_MMC_2_TX = 94,
    DMA_PERIPH_AC97_SURROUND_TX = 95,
    DMA_PERIPH_CENTRE_LFE_TX = 96,
    DMA_PERIPH_NAND_DATA = 97,
    DMA_PERIPH_NAND_CMD = 99,
    DMA_PERIPH_MMC_3_RX = 100,
    DMA_PERIPH_MMC_3_TX = 101,
    DMA_PERIPH_MEM2MEM = 255,
    DMA_PERIPH_FREE = -1
} DMA_PERIPH_ID;

// DMA Peripheral Device DRCMR IDs (legacy definitions)
#define PERIPH_SSP1RX    (13)
#define PERIPH_SSP1TX    (14)

#define PERIPH_SSP2RX    (15)
#define PERIPH_SSP2TX    (16)

#define PERIPH_SSP3RX    (66)
#define PERIPH_SSP3TX    (67)

#define PERIPH_SSP4RX    (71)
#define PERIPH_SSP4TX    (72)

#define PERIPH_CIF1      (68)
#define PERIPH_CIF2      (69)
#define PERIPH_CIF3      (70)

/// DMAC Channel Allocation Priority 
typedef enum
{
   DMA_PRIORITY_HIGH = 0,
   DMA_PRIORITY_MEDIUM,
   DMA_PRIORITY_LOW,
   DMA_PRIORITY_LOWEST
}DMA_CHANNEL_PRIORITY;

// DCMD Register
#define DCMD_INCSRCADDR    (1 << 31)    ///< Source Address Increment
#define DCMD_INCTRGADDR    (1 << 30)    ///< Target Address Increment
#define DCMD_FLOWSRC       (1 << 29)    ///< Source Flow Control
#define DCMD_FLOWTRG       (1 << 28)    ///< Target Flow Control
#define DCMD_CIF_DMA       (1 << 27)    ///< Create a PXA3xx CIF descriptor
#define DCMD_CMPEN         (1 << 25)    ///< Descriptor Compare Enable    
#define DCMD_ADDRMODE      (1 << 23)    ///< Addressing Mode 
#define DCMD_STARTIRQEN    (1 << 22)    ///< Start Interrupt Enable
#define DCMD_ENDIRQEN      (1 << 21)    ///< End Interrupt Enable
#define DCMD_FLYBYS        (1 << 20)    ///< Fly-By Source (PXA270 only)
#define DCMD_FLYBYT        (1 << 19)    ///< Fly-By Target (PXA270 only)

#define DCMD_BSIZE8        (1 << 16)
#define DCMD_BSIZE16       (2 << 16)
#define DCMD_BSIZE32       (3 << 16)

#define DCMD_BWIDTH8       (1 << 14)
#define DCMD_BWIDTH16      (2 << 14)
#define DCMD_BWIDTH32      (3 << 14)

#define DCMD_LENGTH        0x01fff        ///< Length of Transfer in Bytes (MASK)

// DCSR Register 
#define DCSR_RUN           (1 << 31)    ///< Run Bit (read / write)
#define DCSR_NODESC        (1 << 30)    ///< No-Descriptor Fetch (read / write)
#define DCSR_STOPIRQEN     (1 << 29)    ///< Stop Interrupt Enable (when the descriptor is done) (read / write) 
#define DCSR_EORIRQEN      (1 << 28)    ///< End-of-Receive Interrupt Enable
#define DCSR_EORJMPENT     (1 << 27)    ///< Jump to next Descriptor on End-of-Receive
#define DCSR_EORSTOPEN     (1 << 26)    ///< Stop Channel on End-of-Receive
#define DCSR_SETCMPST      (1 << 25)    ///< Set Descriptor Compare Status 
#define DCSR_CLRCMPST      (1 << 24)    ///< Clear Descriptor Compare Status 
#define DCSR_CMPST         (1 << 10)    ///< Descriptor Compare Status
#define DCSR_EORINT        (1 << 9)    ///< End of Receive
#define DCSR_REQPEND       (1 << 8)    ///< Request Pending (read-only)
#define DCSR_RASINTR       (1 << 4)    ///< Request after Channel Stops
#define DCSR_STOPINTR      (1 << 3)    ///< Stop State (read-only)
#define DCSR_ENDINTR       (1 << 2)    ///< End Interrupt (read / write)
#define DCSR_STARTINTR     (1 << 1)    ///< Start Interrupt (read / write)
#define DCSR_BUSERRINTR    (1 << 0)    ///< Bus Error Interrupt (read / write)

#define DRCMR_MAPVLD       (1 << 7)    ///< Valid Channel mapped
#define DDADR_STOP         (1 << 0)

/// Initialisation

//******************************************************************************
/// Returns the library Version.
/// @param[out]         pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild          Returns the build number.         Set this parameter to NULL if not required.          
void DMAGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Initializes the DMA Library.      
void DMAInit(void);

//******************************************************************************
/// De-Initializes the DMA Library.          
void DMADeInit(void);

/// DMA Channel Management

//******************************************************************************
/// Searches for a free channel and allocates it.
/// @param[in]          periph          DMA peripheral device DRCMR ID            
/// @retval             -1              No free channel found.
/// @retval             >=0             Allocated DMA channel number
/// @deprecated Obsolete fuction!
DWORD DMAAllocChannel(DWORD periph);

//******************************************************************************
/// Searches for a free channel with specific priority and allocates it.
/// @param[in]          periph          DMA peripheral device DRCMR ID              
/// @param[in]          priority        Requested Priority. See DMA_CHANNEL_PRIORITY         
/// @retval             -1              No free channel found for the requested priority.
/// @retval             >=0             Allocated DMA channel number.
DWORD DMAAllocChannelEx(DWORD periph, DWORD priority);

//******************************************************************************
/// Release/free DMA Channel
/// @param[in]          chan            DMA channel number.          
/// Remarks:            ATTENTION! Parameter changed to channel number. \n
///                     Used to be peripheral device ID which is inappropriate. \n
///                     See new Function: DMAReleaseDevice.
void DMAReleaseChannel(DWORD chan);

//******************************************************************************
/// Releases Device allocated DMA Channel.
/// @param[in]          periph          DMA peripheral device DRCMR ID  
void DMAReleaseDevice(DWORD periph);

//-----------------------------------------------------------------------------
/// Set a descriptive text for what a DMA channel is used for.
/// @param[in]          chan            DMA channel number
/// @param[in]          szChanUsage     Descriptive Text (Zero-terminated ASCII string)
/// @param[in]          cBytes          String Length in Bytes
void DMASetChannelUsage(DWORD chan, CHAR* szChanUsage, DWORD cBytes);

//-----------------------------------------------------------------------------
/// Get the descriptive text for what a DMA channel is used for.
/// @param[in]          chan            DMA channel number
/// @param[out]         szChanUsage     Descriptive Text (Zero-terminated ASCII string)
/// @param[in]          cBytes          Maximum allowed String Length in Bytes
/// @retval             DWORD           Actually returned string length.
DWORD DMAGetChannelUsage(DWORD chan, CHAR* szChanUsage, DWORD cBytes);
///@}

///@name DMA Preparation
///@{
//-----------------------------------------------------------------------------
/// Create DMA Transfer descriptor. @n
/// NOTE: TransferDescriptors in RAM are only required for transfers of 8kB or more.
/// @param[in]          sadr            Source address
/// @param[in]          tadr            Target address
/// @param[in]          len             Length of Transfer in Bytes, see also DMA Command Registers
/// @param[in]          flags           Flags for the DMA Command Registers
/// @param[in]          loop            TRUE: DMA descriptor chain is a loop
/// @param[out]         pMem            Physical and virtual address of the descriptor
/// @retval             TRUE            Success
/// @retval             FALSE           Error occured
BOOL DMACreateTransferDescriptors(DWORD sadr, DWORD tadr, DWORD len, DWORD flags, BOOL loop, MEM_T* pMem);

//-----------------------------------------------------------------------------
/// Release memory reserved for the transfer descriptor. @n
/// NOTE: TransferDescriptors in RAM are only required for transfers of 8kB or more.
/// @param[in,out]      pMem            Physical and virtual address of the descriptor
/// @retval             TRUE            Success
/// @retval             FALSE           Error occured
BOOL DMADestroyTransferDescriptors(MEM_T* pMem);

//-----------------------------------------------------------------------------
/// Allocate physical memory that can be used as source or target for DMA transfers.
/// @param[out]         pMem            Physical and virual address of the allocated memory
/// @param[in]          size            Required memory size in bytes
/// @retval             TRUE            Success
/// @retval             FALSE           Error occured
BOOL DMAAllocBuffer(MEM_T* pMem, DWORD size);

//-----------------------------------------------------------------------------
/// Free the physical memory that was allocated by DMAAllocBuffer()
/// @param[in,out]      pMem            Physical and virtual address of the allocated memory
/// @retval             TRUE            Success
/// @retval             FALSE           Error occured
BOOL DMAFreeBuffer(MEM_T* pMem);
///@}

///@name DMA Real Time Control
///@{
//-----------------------------------------------------------------------------
/// Start a DMA transfer which was previously setup using DMACreateTransferDescriptors().
/// @param[in]          chan            DMA channel number
/// @param[in]          pMem            Physical and virtual address of the descriptor
/// @param[in]          flags           Flags for the DMA Command Registers
/// @param[in]          block           TRUE=function returns after DMA is finished @n FALSE= function returns immediately
/// @retval             TRUE            Success
/// @retval             FALSE           Timeout detected
DWORD DMAStartTransferEx(DWORD chan, MEM_T* pMem, DWORD flags, BOOL block);

//-----------------------------------------------------------------------------
/// Start DMA transfer non descriptor mode. @n
/// NOTE: This can only be used for transfers < 8kB
/// @param[in]          chan            DMA channel number
/// @param[in]          sadr            Source address
/// @param[in]          tadr            Target address
/// @param[in]          len             Length of Transfer in bytes (max. 8191)
/// @param[in]          cmd             Flags for the DMA Command Registers
/// @param[in]          csr             Flags for the DMA Control/Status Register
void DMAStartShortTransfer(DWORD chan, DWORD sadr, DWORD tadr, DWORD len, DWORD cmd, DWORD csr);

//-----------------------------------------------------------------------------
/// Start DMA transfer non-descriptor mode. @n
/// NOTE: This can only be used for transfers < 8Kb
/// @param[in]          chan            DMA channel number
/// @param[in]          sadr            Source address
/// @param[in]          tadr            Target address
/// @param[in]          len             Length of Transfer in bytes (max. 8191)
/// @param[in]          cmd             Flags for the DMA Command Registers
/// @param[in]          csr             Flags for the DMA Control/Status Register
/// @param[in]          block           TRUE: function returns after DMA is finished @n
///                                     FALSE: function returns immediately
/// @retval        TRUE    Success
/// @retval        FALSE    Timeout detected
DWORD DMAStartShortTransferEx(DWORD chan, DWORD sadr, DWORD tadr, DWORD len, DWORD cmd, DWORD csr, BOOL block);

//-----------------------------------------------------------------------------
/// Start a DMA transfer which was previously setup using DMACreateTransferDescriptors(). @n
/// Needed to be renamed because in Wince 6.0 we have a naming conflict with a CEDDK function!!
/// @param[in]          chan            DMA channel number
/// @param[in]          pMem            Physical and virtual address of the descriptor
/// @param[in]          flags           Flags for the DMA Control/Status Register
void _DMAStartTransfer(DWORD chan, MEM_T* pMem, DWORD flags); 

//-----------------------------------------------------------------------------
/// Wait until DMA transfer is finished
/// @param[in]          chan            DMA channel number
void DMAWaitTransferEnd(DWORD chan);

//-----------------------------------------------------------------------------
/// Early stop DMA transfer
/// @param[in]          chan            DMA channel number
void DMAStopTransfer(DWORD chan);

//-----------------------------------------------------------------------------
/// Get current Target and Source address. @n
//  This can be used to calculate the numbder of bytes already transferred.
/// @param[in]          chan            DMA channel number
/// @param[out]         dtadr           Target address
/// @param[out]         dsadr           Source address
/// @retval             TRUE            Success
/// @retval             FALSE           Error occured
BOOL DMAGetTrgSrcAdd(DWORD chan,DWORD* dtadr, DWORD * dsadr);

//-----------------------------------------------------------------------------
/// Get number of bytes already transferred.
/// @param[in]          chan            DMA channel number
/// @retval             DWORD           Number of bytes already transferred.
DWORD DMAGetLen(DWORD chan);
///@}

#ifdef __cplusplus
}
#endif

#endif