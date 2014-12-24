///
/// @file	     tegra_dma.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: kia $
/// @version     $Rev: 2083 $ 
/// @date        $Date: 2014-03-06 15:05:30 +0100 (Do, 06 Mrz 2014) $
/// @brief	     Tegra specific definitions
/// 
/// @test        T20
/// @target      Colibri Tegra-Based Modules
/// @caveats     None
///     

#ifndef _TEGRA_DMA_H_
#define _TEGRA_DMA_H_

#ifdef __cplusplus
extern "C" {
#endif

/// Explicitly sized signed and unsigned ints.
typedef unsigned char   tTegU8;  ///< 0 to 255
typedef unsigned short  tTegU16; ///< 0 to 65535
typedef unsigned int    tTegU32; ///< 0 to 4294967295
typedef unsigned long   tTegU64; ///< 0 to 18446744073709551615
typedef signed char     tTegS8;  ///< -128 to 127
typedef signed short    tTegS16; ///< -32768 to 32767
typedef signed int      tTegS32; ///< -2147483648 to 2147483647
typedef signed long     tTegS64; ///< 2^-63 to 2^63-1

typedef tTegU8 tTegBool;

/// Tegra Error Type
typedef enum
{
    tegErrorSuccess = 0x0,
    tegErrorForce32 = 0x7FFFFFFF
} tTegError;
typedef enum
{
    TegDfsClockId_Cpu = 1,  /// Specifies CPU clock.
    TegDfsClockId_Avp,      /// Specifies AVP clock.
    TegDfsClockId_System,   /// Specifies system bus clock.
    TegDfsClockId_Ahb,      /// Specifies AHB bus clock.
    TegDfsClockId_Apb,      /// Specifies APB bus clock.
    TegDfsClockId_Vpipe,    /// Specifies video pipe clock.
    TegDfsClockId_Emc,      /// Specifies external memory controller clock.

    TegDfsClockId_Num,
    TegDfsClockId_Force32 = 0x7FFFFFFF
} tTegDfsClockId;

typedef enum
{
    TegDfsBusyHintSyncMode_Async = 1,  /// Asynchronous mode (non-blocking API).
    TegDfsBusyHintSyncMode_Sync,       /// Synchronous mode (blocking API).
    
    TegDfsBusyHintSyncMode_Num,
    TegDfsBusyHintSyncMode_Force32 = 0x7FFFFFFF
} tTegDfsBusyHintSyncMode;

typedef struct tTegDfsBusyHintRec
{
        tTegDfsClockId ClockId;      /// Target clock domain ID. 

    /// Requested boost duration, in milliseconds.
        DWORD BoostDurationMs;

    /// Requested clock frequency level, in kHz.
        DWORD BoostKHz;

    /// Busy pulse mode indicator -- if NV_TRUE, busy boost is completely removed
    /// after busy time has expired; if NV_FALSE, DFS will gradually lower domain
    /// frequency after busy boost.
        tTegBool BusyAttribute;
} tTegDfsBusyHint;

typedef tTegU32 tTegPhysAddr;          ///< Pointer to a physical address

typedef void *tTegDmaHandle;           ///< DMA Handle Type for API

typedef void *tTegDeviceHandle;        ///< Device Handle Type fo API

typedef void *tTegOsSemaphoreHandle;   ///< OS Semaphore Handle Type for API

typedef struct tTegDmaClientBufferRec
{
    /// Specifies the dma source buffer physical address for dma transfer
    tTegPhysAddr sourceBufferPhysicalAddress;

    /// Specifies the dma destination buffer physical address for dma transfer
    tTegPhysAddr destinationBufferPhysicalAddress;

    /// Source address wrap size in bytes. 
    /// Specifies after how many bytes it will be wrapped.
    /// If it is zero then wrapping for source address is disabled
    tTegU32 sourceAddressWrapSize;

    /// Destination address wrap size in bytes.
    /// Specifies after how many bytes it will be wrapped.
    /// If it is zero then wrapping for destination address is disabled
    tTegU32 destinationAddressWrapSize;

    /// Specifies the size of the buffer in bytes which is requested for
    /// transfer
    tTegU32 transferSize;
} tTegDmaClientBuffer;

///
///@brief Specifies the name of modules which can be supported by dma drivers.\n
//These dma modules can be either source or destination based on direction
///
typedef enum
{
    tegDmaModuleIDInvalid = 0x0, ///< Specifies the dma module Id as Invalid
    tegDmaModuleIDMemory,        ///< Specifies the dma module Id for memory
    tegDmaModuleIDI2s,           ///< Specifies the dma module Id for I2s controller
    tegDmaModuleIDAc97,          ///< Specifies the dma module Id for Ac97 controller
    tegDmaModuleIDSpdif,         ///< Specifies the dma module Id for Spdif controller
    tegDmaModuleIDUart,          ///< Specifies the dma module Id for uart controller
    tegDmaModuleIDVfir,          ///< Specifies the dma module Id for Vfir controller
    tegDmaModuleIDMipi,          ///< Specifies the dma module Id for Mipi controller
    tegDmaModuleIDSpi,           ///< Specifies the dma module Id for spi controller
    tegDmaModuleIDSlink,         ///< Specifies the dma module Id for slink controller
    tegDmaModuleIDI2c,           ///< Specifies the dma module Id for I2c controller
    tegDmaModuleIDDvc,           ///< Specifies the dma module Id for Dvc I2c controller
    tegDmaModuleIDAudioApbIf,    ///< Specifies the dma module Id for Audio APB interface controller
    tegDmaModuleIDDtv,           ///< Specifies the dma module Id for Dtv controller
    tegDmaModuleIDOwr,           ///< Specifies the dma module Id for Owr controller
    tegDmaModuleIDHsi,           ///< Specifies the dma module Id for HSI controller
    tegDmaModuleIDMax,           ///< Specifies the maximum number of modules supported
    tegDmaModuleIDNum,
    tegDmaModuleIDForce32 = 0x7FFFFFFF
} tTegDmaModuleID;

///
///@brief Specify the direction of the transfer, either outbound data
///(source -> dest) or inbound data (source <- dest)
///
typedef enum
{
    tegDmaDirectionForward = 0x1, ///< Specifies the direction of the transfer to be srcdevice -> dstdevice
    tegDmaDirectionReverse,       ///< Specifies the direction of the transfer to be dstdevice -> srcdevice
    tegDmaDirectionNum,
    tegDmaDirectionForce32 = 0x7FFFFFFF
} tTegDmaDirection;

///
///@brief Specify the priority of the dma either low priority or high priority
///
typedef enum
{
    tegDmaPriorityLow = 0x1,   ///< Low priority DMA, no guarantee of latency to start transactions
    tegDmaPriorityHigh,        ///< High priority DMA guarantees the first buffer you send will begin immediately
    tegDmaPriorityNum,
    tegDmaPriorityForce32 = 0x7FFFFFFF
} tTegDmaPriority;

#ifdef __cplusplus
}
#endif

#endif //_TEGRA_DMA_H_
