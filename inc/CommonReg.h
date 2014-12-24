///
/// @file	     CommonReg.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1703 $ 
/// @date        $Date: 2013-06-12 14:46:41 +0200 (Mi, 12 Jun 2013) $
/// @brief	     PXA & Tegra Common Register Definitions
/// 
/// @target      PXAxxx, T20
//

#ifndef __COMMON_REG_H__
#define __COMMON_REG_H__

#ifdef __cplusplus
extern "C" {
#endif


/// Register base addresses.
#define REG_BASE				0x40000000
#define PERIPH_REG_BASE			REG_BASE

/// PXA270 and PXA3xx offsets
#define DMA_OFFSET				0x00000000		///< PXA270, PXA3xx: DMA
#define UART1_OFFSET			0x00100000		///< PXA270, PXA3xx: UART1 (FFUART)
#define UART2_OFFSET			0x00200000		///< PXA270, PXA3xx: UART2 (BTUART)
#define I2C_OFFSET				0x00301680		///< PXA270, PXA3xx: I2C
#define AC97_OFFSET				0x00500000		///< PXA270, PXA3xx: AC97
#define USBC_OFFSET				0x00600000		///< PXA270, PXA3xx: USB Client
#define UART3_OFFSET			0x00700000		///< PXA270, PXA3xx: UART3 (STUART)
#define RTC_OFFSET				0x00900000		///< PXA270, PXA3xx: RTC
#define OST_OFFSET				0x00A00000		///< PXA270, PXA3xx: OS Timer
#define PWM0_2_OFFSET			0x00B00000		///< PXA270, PXA3xx: PWM 0 and 2
#define PWM1_3_OFFSET			0x00C00000		///< PXA270, PXA3xx: PWM 1 and 3
#define PWM0_OFFSET				0x00B00000      ///< PXA270, PXA3xx: PWM0
#define PWM1_OFFSET				0x00C00000      ///< PXA270, PXA3xx: PWM1
#define PWM2_OFFSET				0x00B00010      ///< PXA270, PXA3xx: PWM2
#define PWM3_OFFSET				0x00C00010      ///< PXA270, PXA3xx: PWM3
#define INTC_OFFSET				0x00D00000		///< PXA270, PXA3xx: Interrupt Controller
#define GPIO_OFFSET				0x00E00000		///< PXA270, PXA3xx: GPIO
#define SSP1_OFFSET				0x01000000		///< PXA270, PXA3xx: SSP1
#define SD_OFFSET				0x01100000		///< PXA270, PXA3xx: MMC/SDIO
#define KP_OFFSET				0x01500000		///< PXA270, PXA3xx: Keypad
#define USIM_OFFSET				0x01600000		///< PXA270, PXA3xx: USIM
#define SSP2_OFFSET				0x01700000		///< PXA270, PXA3xx: SSP2
#define SSP3_OFFSET				0x01900000		///< PXA270, PXA3xx: SSP3
#define SSP4_OFFSET				0x01A00000		///< PXA270, PXA3xx: SSP4
#define LCD_OFFSET				0x04000000		///< PXA270, PXA3xx: LCD controller
#define USBH_OFFSET				0x0C000000		///< PXA270, PXA3xx: USB Host
#define CIF_OFFSET				0x10000000		///< PXA270, PXA3xx: Camera Interface
#define IMC_OFFSET				0x18000000		///< PXA270, PXA3xx: Internal Memory


/// PXA270 offset.
#define CLK_OFFSET_PXA270		0x01300000		///< PXA270: Clock Manager
#define ARB_OFFSET_PXA270		0x08000048		///< PXA270: System Bus Arbiter
#define MEM_OFFSET				0x08000000		///< PXA270: Memory Controller


/// PXA3xx offset.
#define MFP_OFFSET				0x00E10000		///< PXA3xx: Multi function pin
#define PWR_OFFSET_MH			0x00F00000		///< PXA3xx: Power Management
#define CLK_OFFSET_MH			0x01340000		///< PXA3xx: Clock Manager
#define OWIRE_OFFSET			0x01B00000		///< PXA3xx: One-Wire
#define TOUCH_OFFSET			0x01C00000		///< PXA3xx: Touch-screen
#define CIR_OFFSET				0x01D00000		///< PXA3xx: Consumer Infrared
#define SD2_OFFSET				0x02000000		///< PXA3xx: MMC/SDIO 2
#define USIM2_OFFSET			0x02100000		///< PXA3xx: USIM 2
#define SD3_OFFSET				0x02500000		///< PXA3xx: MMC/SDIO 3
#define NAND_OFFSET				0x03100000		///< PXA3xx: NAND
#define MLCD_OFFSET				0x06000000		///< PXA3xx: Mini LCD
#define ARB_OFFSET				0x0600FE00		///< PXA3xx: System Bus Arbiter
#define PERFDBG_OFFSET			0x0600FF00		///< PXA3xx: Performance Monitoring and Debug
#define DMEM_OFFSET				0x08100000		///< PXA3xx: Dynamic Memory
#define SMEM_OFFSET				0x0A000000		///< PXA3xx: Static Memory
#define GCU_OFFSET				0x14000000		///< PXA3xx: 2D Accelerator
#define VCU_OFFSET				0x16000000		///< PXA3xx: Video Accelerator

/// Tegra.
#define PWM0_OFFSET_TEG			    0x3000A000      ///< T20 + T30: PWM0
#define PWM1_OFFSET_TEG			    0x3000A010      ///< T20 + T30: PWM1
#define PWM2_OFFSET_TEG			    0x3000A020      ///< T20 + T30: PWM2
#define PWM3_OFFSET_TEG			    0x3000A030      ///< T20 + T30: PWM3
#define SLINK_SPI1_OFFSET_TEG	    0x3000D400		///< T20 + T30: SSP1
#define SLINK_SPI2_OFFSET_TEG	    0x3000D600		///< T20 + T30: SSP2
#define SLINK_SPI3_OFFSET_TEG	    0x3000D800		///< T20 + T30: SSP3
#define SLINK_SPI4_OFFSET_TEG	    0x3000DA00		///< T20 + T30: SSP4
#define SLINK_SPI5_OFFSET_TEG	    0x3000DC00		///< T30: SSP5
#define SLINK_SPI6_OFFSET_TEG	    0x3000DE00		///< T30: SSP6
#define SERIAL_FLASH_SPI_OFFSET_TEG	0x3000C380		///< T20 + T30: Serial flash spi
#define I2C1_OFFSET_TEG				0x3000C000		///< T20 + T30: I2C1
#define I2C2_OFFSET_TEG				0x3000C400		///< T20 + T30: I2C2
#define I2C3_OFFSET_TEG				0x3000C500		///< T20 + T30: I2C3

#ifdef __cplusplus
}
#endif

#endif  // __COMMON_REG_H__