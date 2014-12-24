///
/// @file        intlib.h
/// @copyright   Copyright (c) 2013 Toradex AG
/// $Author: kia $
/// $Rev: 2175 $ 
/// $Date: 2014-04-22 10:24:26 +0200 (Di, 22 Apr 2014) $
/// @brief       Header file for the Interrupt Library
/// 
/// @target      PXAxxx, Txx 
/// 

#ifndef _INTLIB_H_
#define _INTLIB_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#define INT_VER_MAJ     2
#define INT_VER_MIN     5
#define INT_VER_BUILD   0

/// Defines for Interrupt Edge Detect
#define GPIO_EDGE_RISING  1
#define GPIO_EDGE_FALLING 2

/// Defines for Interrupt Level Detect
#define GPIO_LEVEL_HIGH (GPIO_EDGE_RISING | 4)
#define GPIO_LEVEL_LOW  (GPIO_EDGE_FALLING | 4)

/// Tegra IRQ numbers
typedef enum
{
    // ARM specific interrupts, documented in ARM's Cortex-A9 MPCore Technical Reference Manual
    TEG_IRQ_SGI0              =   0, TEG_IRQ_SGI1,                TEG_IRQ_SGI2,                TEG_IRQ_SGI3,                TEG_IRQ_SGI4,                TEG_IRQ_SGI5,                TEG_IRQ_SGI6,                TEG_IRQ_SGI7,
    TEG_IRQ_SGI8              =   8, TEG_IRQ_SGI9,                TEG_IRQ_SGI10,               TEG_IRQ_SGI11,               TEG_IRQ_SGI12,               TEG_IRQ_SGI13,               TEG_IRQ_SGI14,               TEG_IRQ_SGI15,
    TEG_IRQ_GLOBAL_TIMER      =  27, 
    TEG_IRQ_LEGACY_FIQ        =  28,
    TEG_IRQ_PRIVATE_TIMER     =  29,
    TEG_IRQ_WATCHDOG_TIMERS   =  30,
    TEG_IRQ_LEGACY_IRQ        =  31,
    // Tegra: Primary Interrupt Controller
    TEG_IRQ_TMR1              =  32, TEG_IRQ_TMR2,                TEG_IRQ_RTC,                 TEG_IRQ_I2S2,                TEG_IRQ_SHR_SEM_INBOX_FULL,  TEG_IRQ_SHR_SEM_INBOX_EMPTY, TEG_IRQ_SHR_SEM_OUTBOX_FULL, TEG_IRQ_SHR_SEM_OUTBOX_EMPTY,
    TEG_IRQ_VDE_UCQ           =  40, TEG_IRQ_VDE_SYNC_TOKEN,      TEG_IRQ_VDE_BSEV,            TEG_IRQ_VDE_BSEA,            TEG_IRQ_VDE_SXE,             TEG_IRQ_I2S1,                TEG_IRQ_SDMMC1,              TEG_IRQ_SDMMC2,              
    TEG_IRQ_XIO               =  48, TEG_IRQ_VDE,                 TEG_IRQ_AVP_UCQ,             TEG_IRQ_SDMMC3,              TEG_IRQ_USB,                 TEG_IRQ_USB2 ,               TEG_IRQ_SDMMC4_DUCPLICATE1,  TEG_IRQ_EIDE,                
    TEG_IRQ_NANDCTRL          =  56, TEG_IRQ_VCP,                 TEG_IRQ_APB_DMA_CPU,         TEG_IRQ_AHB_DMA_CPU,         TEG_IRQ_ARB_SEM_GNT_CO,      TEG_IRQ_ARB_SEM_GNT_CP,      TEG_IRQ_OWR,                 TEG_IRQ_SDMMC4_DUPLICATE2,             
    // Tegra: Second Interrupt Controller
    TEG_IRQ_GPIO1             =  64, TEG_IRQ_GPIO2,               TEG_IRQ_GPIO3,               TEG_IRQ_GPIO4,               TEG_IRQ_UART1,               TEG_IRQ_UART2,               TEG_IRQ_I2C,                 TEG_IRQ_SPI,                 
    TEG_IRQ_TWC               =  72, TEG_IRQ_TMR3,                TEG_IRQ_TMR4,                TEG_IRQ_FLOW_RSM_CPU,        TEG_IRQ_FLOW_RSM_COP,        TEG_IRQ_SPDIF,               TEG_IRQ_UART3,               TEG_IRQ_MIPI_HS,             
    TEG_IRQ_EVENT_GPIO_A      =  80, TEG_IRQ_EVENT_GPIO_B,        TEG_IRQ_EVENT_GPIO_C,        TEG_IRQ_EVENT_GPIO_D,        TEG_IRQ_VFIR,                TEG_IRQ_DVC,                 TEG_IRQ_STAT_MON,            TEG_IRQ_GPIO5,               
    TEG_IRQ_CPU0_PMU_INTR     =  88, TEG_IRQ_CPU1_PMU_INTR,       TEG_IRQ_NONE,                TEG_IRQ_SBC1 ,               TEG_IRQ_APB_DMA_COP,         TEG_IRQ_AHB_DMA_COP,         TEG_IRQ_DMA_TX,              TEG_IRQ_DMA_RX,              
    // Tegra: Third Interrupt Controller
    TEG_IRQ_HOST1X_SYNCPT_COP =  96, TEG_IRQ_HOST1X_SYNCPT_CPU,   TEG_IRQ_HOST1X_GEN_COP,      TEG_IRQ_HOST1X_GEN_CPU,      TEG_IRQ_MPE,                 TEG_IRQ_VI,                  TEG_IRQ_EPP,                 TEG_IRQ_ISP,                 
    TEG_IRQ_GR2D              = 104, TEG_IRQ_DISPLAY,             TEG_IRQ_DISPLAYB,            TEG_IRQ_HDMI,                TEG_IRQ_TVO,                 TEG_IRQ_MC,                  TEG_IRQ_EMC ,                TEG_IRQ_NONE2,               
    TEG_IRQ_NOR_FLASH         = 112, TEG_IRQ_AC97,                TEG_IRQ_SBC2,                TEG_IRQ_SBC3,                TEG_IRQ_I2C2,                TEG_IRQ_KBC,                 TEG_IRQ_PMU_EXT,             TEG_IRQ_GPIO6,               
    TEG_IRQ_TVDAC             = 120, TEG_IRQ_GPIO7,               TEG_IRQ_UART4,               TEG_IRQ_UART5,               TEG_IRQ_I2C3,                TEG_IRQ_SBC4,                TEG_IRQ_NONE3,               TEG_IRQ_SW_INTR,             
    // Tegra: Fourth Interrupt Controller
    TEG_IRQ_SNOR              = 128, TEG_IRQ_USB3,                TEG_IRQ_PCIE_INT,            TEG_IRQ_PCIE_MSI,            TEG_IRQ_PCIE_WAKE,           TEG_IRQ_NONE4,               TEG_IRQ_NONE5,               TEG_IRQ_NONE6,               
    TEG_IRQ_APB_DMA_CH0       = 136, TEG_IRQ_APB_DMA_CH1,         TEG_IRQ_APB_DMA_CH2,         TEG_IRQ_APB_DMA_CH3,         TEG_IRQ_APB_DMA_CH4,         TEG_IRQ_APB_DMA_CH5,         TEG_IRQ_APB_DMA_CH6,         TEG_IRQ_APB_DMA_CH7,         
    TEG_IRQ_APB_DMA_CH8       = 144, TEG_IRQ_APB_DMA_CH9 ,        TEG_IRQ_APB_DMA_CH10,        TEG_IRQ_APB_DMA_CH11,        TEG_IRQ_APB_DMA_CH12,        TEG_IRQ_APB_DMA_CH13,        TEG_IRQ_APB_DMA_CH14,        TEG_IRQ_APB_DMA_CH15,        
    TEG_IRQ_NONE7             = 152, TEG_IRQ_NONE8,               TEG_IRQ_NONE9,               TEG_IRQ_NONE10,              TEG_IRQ_NONE11,              TEG_IRQ_NONE12,              TEG_IRQ_NONE13,              TEG_IRQ_NONE14,              
} tegraIrqNumbers;

#ifdef KERNEL_MODE // in kernel mode we use the native os functions instead of the ...Compat ones.
#define LoadIntChainHandlerCompat LoadIntChainHandler
#define FreeIntChainHandlerCompat FreeIntChainHandler
#define InterruptInitializeCompat InterruptInitialize
#define InterruptDisableCompat InterruptDisable
#define InterruptDoneCompat InterruptDone
#define CreateStaticMappingCompat CreateStaticMapping
#define KernelLibIoControlCompat KernelLibIoControl
#else 

//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of LoadIntChainHandler()
/// @param[in]            lpFilename       Pointer to Filename
/// @param[in]            lpszFunctionName Function Name String
/// @param[in]            bIRQ             IRQ Number
/// @retval               HANDLE           Instance Handle
HANDLE LoadIntChainHandlerCompat(LPCWSTR lpFilename, LPCWSTR lpszFunctionName, BYTE bIRQ);

//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of FreeIntChainHandler()
/// @param[in]            hInstance       Instance Handle       
/// @retval               True            Success
/// @retval               False           Failure
BOOL FreeIntChainHandlerCompat(HANDLE hInstance);

//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of InterruptInitialize()
/// @param[in]            idInt           Interrupt ID  
/// @param[in]            hEvent          Event Handle
/// @param[in]            pvData          Pointer to data buffer
/// @param[in]            cbData          Size of data buffer
/// @retval               True            Success
/// @retval               False           Failure
BOOL InterruptInitializeCompat(DWORD idInt, HANDLE hEvent, LPVOID pvData, DWORD cbData);

//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of InterruptDisable()
/// @param[in]            idInt           Interrupt ID     
void InterruptDisableCompat(DWORD idInt);

//
//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of Interrupt()
/// @param[in]            idInt           Interrupt ID       
void InterruptDoneCompat(DWORD idInt);

//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of CreateStaticMapping()
/// @param[in]            dwPhysBase      Physical Base
/// @param[in]            dwSize          Size
/// @retval               Virtual address
LPVOID CreateStaticMappingCompat(DWORD dwPhysBase, DWORD dwSize);

//******************************************************************************
/// Win CE 5.0/6.0/7.0 compatible version of KernelLibIoControl()
/// @param[in]            hModule         Module Handle returned by LoadIntChainHandlerCompat()
/// @param[in]            dwIoControlCode IO Control Code
/// @param[in]            lpInBuf         Pointer to Input Buffer
/// @param[in]            nInBufSize      Input Buffer Size in bytes
/// @param[in]            lpOutBuf        Pointer to Output Buffer
/// @param[in]            nOutBufSize     Output Buffer Size
/// @param[out]           lpBytesReturned Number of bytes returned
/// @retval               True            Success
/// @retval               False           Failure
BOOL KernelLibIoControlCompat(HANDLE hModule, DWORD dwIoControlCode, LPVOID lpInBuf, DWORD nInBufSize, LPVOID lpOutBuf, DWORD nOutBufSize, LPDWORD lpBytesReturned);

#endif // KERNEL_MODE
// The following functions are also defined in KERNEL MODE

//******************************************************************************
/// Get the IRQ for a specific GPIO Pin
/// @param[in]            dwGpio          GPIO Pin Number
/// @retval               IRQ Number
DWORD GetGPIOIrq(DWORD dwGpio);

//******************************************************************************
/// Set the GPIO Interrupt Edge Detect
/// @param[in]            dwGpio          GPIO Pin Number       
/// @param[in]            dwEdge          GPIO_EDGE_RISING or GPIO_EDGE_FALLING      
/// @retval               True            Success
/// @retval               False           Failure
BOOL SetGPIOIrqEdge(DWORD dwGpio, DWORD dwEdge);

//******************************************************************************
/// Get the System Interrupt Number for the specified IRQ
/// @param[in]            dwIRQ           IRQ Number
/// @retval               System Interrupt Number
DWORD RequestSysInterrupt(DWORD dwIrq);

//******************************************************************************
/// Release the specified system interrupt
/// @param[in]            dwSysIntr       System Interrupt Number       
/// @retval               True            Success
/// @retval               False           Failure
BOOL ReleaseSysIntr(DWORD dwSysIntr);

//******************************************************************************
/// Outputs library version
/// @param[out] verMaj    Major version number. Set this parameter to NULL if not required
/// @param[out] verMin    Minor version number. Set this parameter to NULL if not required
/// @param[out] build     Build number.         Set this parameter to NULL if not required
void INTGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _INTLIB_H_
