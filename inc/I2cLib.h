///
/// @file        I2cLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 2160 $ 
/// $Date: 2014-04-15 12:43:35 +0200 (Di, 15 Apr 2014) $
/// @brief       I2C Library\n
///                     I2c uses system interrupt #18\n
///                     The following flags can be set to act as interrupt sources:\n
///                     - ICR:SAD    Slave Address Detected\n
///                     - ICR:GCAD   Global Call Address Detected (interrupt enabled by SADIE flag)\n
///                     - ICR:ALD    Arbitration Loss Detected while in master mode\n
///                     - ICR:SSD    Slave Stop Detected while in slave mode\n
///                     - ICR:BED    Bus Error Detected (NAK received on transmitting a byte in master mode)\n
///                     - ICR:IRF    IDBR Receive Full\n
///                     - ICR:ITE    IDBR Transmit Empty
/// 
/// @test        PXA320, WinCE 5.0
/// @target      PXAxxx, T20
/// 

#ifndef __I2CINT_LIB_H__
#define __I2CINT_LIB_H__

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2CLIB_VER_MAJ      4    ///< Main Version of this library 
#define I2CLIB_VER_MIN      1    ///< Subversion of this library
#define I2CLIB_VER_BUILD    1    ///< Build Number of this library

/// use this value in the I2CBurstRead/Write functions to avoid sending an offset byte
#define I2C_OFFSET_NONE        (255)

/// Specification of possible i2c data rates
typedef enum
{
    I2C_100KBPS, I2C_400KBPS, I2C_UNDEF_KBPS
} I2CSpeed;

//******************************************************************************
/// Returns the library Version.
/// @param[out]         pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild          Returns the build number.         Set this parameter to NULL if not required.
void I2cIntGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Returns the library Version - for compatibility
/// @param[out]         pVerMaj         Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]         pVerMin         Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]         pBuild          Returns the build number.         Set this parameter to NULL if not required.
void I2CGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Syncronisation function if I2C bus is used in multiple threads
/// (e.g. RTCSync which is loaded by default)
/// @param[in]    timeout        Maximum time [ms] to wait for the I2C Mutex
/// @retval TRUE  Success
/// @retval FALSE See GetLastError().\n
///               possible Errors: WAIT_FAILED
BOOL GetI2CLock(DWORD timeout);

//******************************************************************************
/// Release I2C bus for other threads
/// (e.g. RTCSync which is loaded by default)
/// @retval TRUE  Success
/// @retval FALSE Unspecified Error. See GetLastError().
BOOL ReleaseI2CLock();

//******************************************************************************
/// Initialization to use I2C bus\n
/// Use this function instead of I2CInit() if you need to use another i2c bus than the default bus.
/// @param[in] I2CRegAddr    Base address of the CPU's i2c controller.
/// @retval TRUE  Success
/// @retval FALSE Unspecified Error. See GetLastError().
BOOL I2CInitEx(DWORD I2CRegAddr);

//******************************************************************************
/// Initialization to use I2C bus\n
/// Enable Clock, configure GPIOs, map registers, setup I2C interrupt
/// @retval        TRUE  success
/// @retval        FALSE See GetLastError(). Possible error codes:\n
///                      ERROR_INVALID_PARAMETER\n
///                      ERROR_NO_SYSTEM_RESOURCES\n
///                      ERROR_INVALID_COMPUTERNAME
BOOL I2CInit(void);

//******************************************************************************
/// DeInit I2C bus\n
/// Release I2C interrupt, unmap I2C registers and deinit Clock
void I2CDeInit(void);

//******************************************************************************
/// Set I2C bus speed
/// @param[in]    Speed: I2C_100KBPS or I2C_400KBPS
void I2CSetSpeed (I2CSpeed Speed);

//******************************************************************************
/// Write data to one or multiple registers of an i2c device.\n
/// The function assumes that the i2c protocol defines the first transmitted byte to be the device's register offset.
/// @param[in]    uSlaveAddress    The device's i2c address (0..127)
/// @param[in]    pBuffer            Pointer to the data bytes to Transmit
/// @param[in]    ucOffset        The device's register address (offset). This is one byte transmitted after the slave address.\n
///                             Set to any negative value if *no* offset byte should be transmitted
/// @param[in]    iNumberBytes    Number of Bytes to Transmit
/// @retval        TRUE  success
/// @retval        FALSE See GetLastError(). Possible error codes:\n
///                      ERROR_BUSY\n
///                      ERROR_INVALID_PARAMETER\n
///                      ERROR_TIMEOUT\n
///                      ERROR_IO_DEVICE\n
///                      ERROR_INVALID_ADDRESS\n 
///                      ERROR_OPERATION_ABORTED\n
///                   On Tegra 3, a byte not acknowledged will be reported with a Timeout Error rather than\n
///                   an Operation Aborted Error
BOOL I2CBurstWrite(unsigned char uSlaveAddress, unsigned char *pBuffer, short ucOffset, unsigned iNumberBytes );

//******************************************************************************
/// Read data from one or multiple registers of an i2c device.\n
/// The function assumes that the i2c protocol defines the first transmitted byte to be the device's register offset.
/// @param[in]    uSlaveAddress    The device's i2c address (0..127)
/// @param[in]    pBuffer            Pointer to the data bytes to Receive
/// @param[in]    ucOffset        The device's register address (offset). This is one byte transmitted after the slave address.\n
///                             Set to any negative value if *no* offset byte should be transmitted
/// @param[in]    iNumberBytes    Number of Bytes to Receive
/// @retval        TRUE  success
/// @retval        FALSE Error during Transmission
/// @retval        FALSE See GetLastError(). Possible error codes:\n
///                      ERROR_BUSY\n
///                      ERROR_INVALID_PARAMETER\n
///                      ERROR_TIMEOUT\n
///                      ERROR_IO_DEVICE\n
///                      ERROR_INVALID_ADDRESS\n
///                      ERROR_OPERATION_ABORTED
BOOL I2CBurstRead(unsigned char uSlaveAddress, unsigned char *pBuffer, short ucOffset, unsigned iNumberBytes );

#ifdef __cplusplus
}
#endif

#endif // __I2CINT_LIB_H__