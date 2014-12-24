///
/// @file         spilib.h
/// @copyright    Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Revision: 2293 $
/// $Date: 2014-05-27 16:40:57 +0200 (Di, 27 Mai 2014) $
/// @target       Colibri Modules
/// @test         Tested on:  Colibri PXAxxx, T20
/// @brief        Library to manage access to the Colibri's SPI ports
///
/// if SPIDMADEF is set in the preprocessor, the library is generated using DMA.
///
/// Clock setting is ignored when Slave mode selected
///
///  Frequency |  T20  | PXA320 | PXA310 | PXA300 | PXA270*
///  --------- | :---: | :----: | :----: | :----: | :----:
///  54MHz     |   Y   |    -   |    -   |    -   |    - 
///  36MHz     |   Y   |    -   |    -   |    -   |    -
///  27MHz     |   Y   |    -   |    -   |    -   |    -
///  26MHz     |   Y   |    -   |    Y   |    -   |    -
///  18MHz     |   Y   |    -   |    -   |    -   |    -
///  13MHz     |   Y   |    Y   |    Y   |    Y   |    Y   
///  12MHz     |   Y   |    -   |    -   |    -   |    -
///  6.5MHz    |   Y   |    Y   |    Y   |    Y   |    Y
///  6MHz      |   Y   |    -   |    -   |    -   |    -
///  4MHz      |   Y   |    -   |    -   |    -   |    -
///  3MHz      |   Y   |    -   |    -   |    -   |    -
///  2MHz      |   Y   |    -   |    -   |    -   |    -
///  1MHz      |   Y   |    -   |    -   |    -   |    Y
///  0.812MHz  |   Y   |    Y   |    Y   |    Y   |    Y
///  0.2MHz    |   Y   |    -   |    Y   |    -   |    Y
///  0.1MHz    |   Y   |    Y   |    Y   |    Y   |    Y
///  0.026MHz  |   Y   |    -   |    -   |    -   |    -
///
///  *The Colibri PXA270 module allows much more frequency settings. Allowed are all 13MHz/x values
///  where x is a value from 1 to 4096.
///
/// CPOL 0, CPHA 0 - data is captured on clock rising edge, base value of clock is 0 \n
/// CPOL 0, CPHA 1 - data is captured on clock falling edge, base value of clock is 0 \n
/// CPOL 1, CPHA 0 - data is captured on clock falling edge, base value of clock is 1 \n
/// CPOL 1, CPHA 1 - data is captured on clock rising edge, base value of clock is 1 \n
///
/// SODIMM Pins Initialisation Structure - can be used for T20 modules. \n
/// The library will validate the SODIMM pin against the SPI Instance.  \n
/// If valid, the library will set the correct alternate function for the supplied SODIMM pin for the SPI Port.  \n 
/// The library assumes that the supplied configuration combination has no pinmux group conflicts. \n
/// The library assumes that the user has ensured all pre-existing pin setups have already been cleared. \n
/// For valid options for SPI Signal lines please see the T20 Datasheet 
///
/// If you require different Pin allocations on a PXA module then, after SPIINIT has been called, \n
/// clear the default settings and configure your required settings prior to using any other functions.
///
/// If Pin Configuration is passed as a null pointer in SPIInit, then the library will use the default settings. \n
/// These are:
///
///  | Function   |      |  T20   |          |      | PXA270 |          |      | PXA300 |          |      | PXA310 |          |      | PXA320 |          |
///  | ---------- | :--: | :----: | :------: | :--: | :----: | :------: | :--: | :----: | :------: | :--: | :----: | :------: | :--: | :----: | :------: |
///  |            | GPIO | SODIMM | Eval 3.1 | GPIO | SODIMM | Eval 3.1 | GPIO | SODIMM | Eval 3.1 | GPIO | SODIMM | Eval 3.1 | GPIO | SODIMM | Eval 3.1 |
///  | SPI1 CLK   | X5/1 |  100   |   X9-7   | 23/2 |   88   |  X10-33  | 85/1 |   88   |  X10-33  | 85/1 |   88   |  X10-33  | 83/1 |   88   |  X10-33  |
///  | SPI1 MOSI  | T2/3 |   69   |  X22-21  | 25/2 |   92   |  X10-35  | 88/6 |   92   |  X10-35  | 88/6 |   92   |  X10-35  | 86/6 |   92   |  X10-35  |
///  | SPI1 MISO  | T3/3 |   77   |  X22-22  | 26/1 |   90   |  X10-34  | 87/6 |   90   |  X10-34  | 87/6 |   90   |  X10-34  | 85/6 |   90   |  X10-34  |
///  | SPI1 CS0   | X6/1 |  102   |   X9-9   | 24/2 |   86   |  X10-32  | 86/1 |   86   |  X10-32  | 86/1 |   86   |  X10-32  | 84/1 |   86   |  X10-32  |
///  |            |      |        |          |      |        |          |      |        |          |      |        |          |      |        |          |
///  | SPI2 CLK   | A7/3 |   67   |  X10-30  | 19/1 |   55   |   X9-5   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  | SPI2 MOSI  | D0/3 |   88   |  X10-33  | 38/2 |   37   |  X10-12  | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  | SPI2 MISO  | D1/3 |   90   |  X10-34  | 11/2 |   28   |  X10-27  | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  | SPI2 CS0   | D3/3 |   86   |  X10-32  | 14/2 |   63   |   X9-6   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  |            |      |        |          |      |        |          |      |        |          |      |        |          |      |        |          |
///  | SPI3 CLK   | A6/3 |   30   |  X10-28  | 52/2 |   73   |  X10-31  | 91/1 |  154   |  X3-C7   | 91/1 |  154   |  X3-C7   | 69/2 |   66   |  X9-31   |
///  | SPI3 MOSI  | B6/3 |   55   |   X9-5   | 35/3 |  135   |  X10-40  | 93/1 |  158   |  X3-C8   | 93/1 |  158   |  X3-C8   | 71/2 |   57   |  X9-33   |
///  | SPI3 MISO  | B7/3 |   63   |   X9-6   | 82/1 |   77   |  X22-22  | 94/1 |  160   |  X3-A9   | 94/1 |  160   |  X3-A9   | 72/2 |   61   |  X9-34   |
///  | SPI3 CS0   | B5/3 |   28   |  X10-27  | 83/1 |   79   |  X22-11  | 92/1 |  156   |  X3-A8   | 92/1 |  156   |  X3-A8   | 70/2 |   64   |  X9-32   |
///  |            |      |        |          |      |        |          |      |        |          |      |        |          |      |        |          |
///  | SPI4 CLK   | D0/1 |   88   |  X10-33  | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  | SPI4 MOSI  | D4/1 |   92   |  X10-35  | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  | SPI4 MISO  | D1/1 |   90   |  X10-34  | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///  | SPI4 CS0   | D3/1 |   86   |  X10-32  | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   | n.a. |  n.a.  |   n.a.   |
///
///  n.b. Note the conflict in the T20 default settings between SPI2 & SPI4
///
///  SPI4 on T20 is SPI1 on all PXAxxx modules
///
///  On power up for T20, SODIMM 104 is configured as SPI3 MISO and SODIMM 131 is configured as SPI3 CS0
///  and hence will need to be cleared if SPI3 is to be used in anything other than default settings. 

#ifndef _SPILIB_H_
#define _SPILIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <windows.h>

#define SPI_VER_MAJ     5            ///< Library Major Version Number
#define SPI_VER_MIN     2            ///< Library Minor Version Number
#define SPI_VER_BUILD   0            ///< Library Build Number  

#define SPI_MAX_TIMEOUT      (2000)  ///< Maximum timeout in milliseconds
#define SPI_INFINITE_TIMEOUT (0)     ///< 0 = Infinite (no) timeout

#define SPI_PXA_MAX_CHANNELS (4)     ///< Maximum available PXA SPI Channels
#define SPI_T20_MAX_CHANNELS (4)     ///< Maximum available T20 SPI Channels
#define SPI_T30_MAX_CHANNELS (6)     ///< Maximum available T20 SPI Channels

#define SPI_MAX_CHANNELS     (6)     ///< Maximum SPI channels across all modules 

///@{
#define SPI_CLOCK_54_MHZ  54000      ///< SPI Clock Constant for 54MHz
#define SPI_CLOCK_36_MHZ  36000      ///< SPI Clock Constant for 36MHz
#define SPI_CLOCK_27_MHZ  27000      ///< SPI Clock Constant for 27MHz
#define SPI_CLOCK_26_MHZ  26000      ///< SPI Clock Constant for 26MHz
#define SPI_CLOCK_18_MHZ  18000      ///< SPI Clock Constant for 18MHz
#define SPI_CLOCK_13_MHZ  13000      ///< SPI Clock Constant for 13MHz
#define SPI_CLOCK_12_MHZ  12000      ///< SPI Clock Constant for 12MHz
#define SPI_CLOCK_6_5_MHZ  6500      ///< SPI Clock Constant for 6.5MHz
#define SPI_CLOCK_6_MHZ    6000      ///< SPI Clock Constant for 6MHz
#define SPI_CLOCK_4_MHZ    4000      ///< SPI Clock Constant for 4MHz
#define SPI_CLOCK_3_MHZ    3000      ///< SPI Clock Constant for 3MHz
#define SPI_CLOCK_2_MHZ    2000      ///< SPI Clock Constant for 2MHz
#define SPI_CLOCK_1_MHZ    1000      ///< SPI Clock Constant for 1MHz
#define SPI_CLOCK_812_KHZ   812      ///< SPI Clock Constant for 812KHz
#define SPI_CLOCK_200_KHZ   200      ///< SPI Clock Constant for 200KHz
#define SPI_CLOCK_100_KHZ   100      ///< SPI Clock Constant for 100KHz
#define SPI_CLOCK_26_KHZ     26      ///< SPI Clock Constant for 26KHz

///@}

#define SPI_CLOCK_MAX     54000      ///< Maximum SPI Clock for Tegra
#define SPI_CLOCK_MIN        26      ///< Minimum SPI Clock for Tegra

#define SPI_MASTER        FALSE      ///< Configure as Master Device
#define SPI_SLAVE          TRUE      ///< Configure as Slave Device

#define SPI_MODE_0            0      ///< Mode 0 - Polarity (CPOL) 0, Phase (CPHA) 0
#define SPI_MODE_1            1      ///< Mode 1 - Polarity (CPOL) 0, Phase (CPHA) 1
#define SPI_MODE_2            2      ///< Mode 2 - Polarity (CPOL) 1, Phase (CPHA) 0
#define SPI_MODE_3            3      ///< Mode 3 - Polarity (CPOL) 1, Phase (CPHA) 1

/*
/// define possible options to specify an IO pin
typedef enum 
{
    ioNone              =     -1,  // specify that this Io is unused and ignored
    ioDefault           = 0x0000,  // undefined usage (could be GPIO, SODIMM, ...)
    ioGpio              = 0x0010,  // specify a GPIO
    ioColibriPin        = 0x0020,  // specify an SODIMM pin of a Colibri module
    ioColibriExtension  = 0x0021,  // specify a pin on the Colibri FFC Extension connector
    ioApalisPin         = 0x0030   // specify an MXM3 pin of an Apalis module
} tIoType;

/// Define one IO on one Toradex module
/// The struct below can also be seen as a DWORD 
///  - High word = ioType 
///  - Low word  = gpio / pin number
typedef struct 
{
    union                                      
    {
        unsigned int Io                 :16;    // unspecified IO - could be Gpio, Sodimm, MXM3 Pin, ...   
        unsigned int Gpio               :16;    
        unsigned int ColibriPin         :16;
        unsigned int ColibriExtensionPin:16;
        unsigned int ApalisPin          :16;
    };
    tIoType Type :16;                            // define meaning of the number above: Gpio, Sodimm pin, ...
} tIO;

typedef struct   
{
    tIO ClkSodimmPin;   ///< SODIMM Pin for Clock Signal
    tIO MOSISodimmPin;  ///< SODIMM Pin for MOSI Signal
    tIO MISOSodimmPin;  ///< SODIMM Pin for MISO Signal
    tIO CSSodimmPin;    ///< SODIMM Pin for Chip Select. Initialise to '-1' if not required
} xxtSpiPinConfiguration;   ///< Contains Pin Configuration for SPI Port 

//const xxtSpiPinConfiguration cfg = {{15, ioGpio}, {33, ioApalisPin}, {32, ioGpio}, {0, ioNone}};
*/

/// Pin Configuration data structure for SPI Port
typedef struct SPIPinConfiguration  
{
    DWORD ClkSodimmPin;   ///< SODIMM Pin for Clock Signal
    DWORD MOSISodimmPin;  ///< SODIMM Pin for MOSI Signal
    DWORD MISOSodimmPin;  ///< SODIMM Pin for MISO Signal
    DWORD CSSodimmPin;    ///< SODIMM Pin for Chip Select. Initialise to '-1' if not required
} tSpiPinConfiguration;   ///< Contains Pin Configuration for SPI Port 

typedef enum 
{
    SpiWaitByInterrupt,     ///< better overall system performance, bug increased Spi latency
    SpiWaitByBusyLoop       ///< minimum Spi latency, but occupies CPU while waiting.
} tSpiWaitMethod;

//******************************************************************************
/// Outputs library version
/// @param[out] verMaj          major version number. Set this parameter to NULL if not required
/// @param[out] verMin          minor version number. Set this parameter to NULL if not required
/// @param[out] build           build number.         Set this parameter to NULL if not required
void SPIGetLibVersion(DWORD *verMaj, DWORD *verMin, DWORD *build);

//******************************************************************************
/// Initializes SPI unit
/// @param[in]  spiChannel        Selected SPI channel (1 to SPI_CHANNELS_MAX)
/// @param[in]  dataBits          Number of Databits [1-32]
/// @param[in]  clockFrequency    Select from Clock Frequency defines above
/// @param[in]  isSlave           True = Slave, False = Master
/// @param[in]  mode              SPI Mode - Select from Mode defines above
/// @param[in]  pinConfiguration  Pointer to SpiPinConfiguration structure,\n
///                               Set pointer to NULL to use default settings.\n
///                               Set unused pins to '-1'.
/// @retval     TRUE    Success
/// @retval     FALSE   Failure
BOOL SPIInit(DWORD spiChannel, DWORD dataBits, DWORD clockFrequency, BOOL isSlave, DWORD mode, tSpiPinConfiguration *pinConfiguration);

//******************************************************************************
/// De-Initializes SPI unit.\n
/// Calling this function is very important for cleaning up.\n
/// Otherwise SPI interrupts will be blocked, and can only be re-activated by resetting the system
/// @param[in]  spiChannel     Selected SPI channel (1 - SPI_CHANNELS_MAX)
void SPIDeinit(DWORD spiChannel);

//******************************************************************************
/// Define how the library shall wait to read / write the FIFO.\n
/// @param[in]  spiChannel    Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[in]  method        "SpiWaitByInterrupt" (Default): increases Spi latency, but leads to low CPU load
///                           "SpiWaitByBusyLoop":  provides the best SPI performance, but loads the CPU
void TegSpiSetWaitMethod(DWORD spiChannel, tSpiWaitMethod method);

#ifdef SPIDMADEF

//******************************************************************************
/// Simultaneously reads and writes data from/to SPI using DMA.\n
/// The function returns after buffer is full or when a timeout occurs.
/// @param[in]  spiChannel      Selected SPI channel (1 to SPI_CHANNELS_MAX)
/// @param[out] rxBuffer        Virtual Address of the receive buffer
/// @param[in]  txBuffer        Virtual Address of the transmit buffer
/// @param[in]  numberOfFrames  Number of SPI Frames to transmit and receive - must be DWORD aligned for PXA
/// @param[in]  timeout         The function returns after this timeout. [1 to SPI_MAX_TIMEOUT milliseconds, 0 = infinite]
/// @param[in]  chipSelectID    ID of the required Chip Select, ignored in slave mode, ignored for PXA        
/// @retval     >0              Number of bytes received
/// @retval     0               Error
DWORD SPIReadWrite(DWORD spiChannel, DWORD *rxBuffer, DWORD *txBuffer, DWORD numberOfFrames, DWORD timeout, DWORD chipSelectID);

#endif

//******************************************************************************
/// Writes data to and Reads data from SPI in polling mode
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[in]  rxBuffer        Pointer to the transmit buffer. NULL if you want to transmit only.
/// @param[in]  txBuffer        Pointer to the receive buffer. NULL if you want to receive only.
/// @param[in]  numberOfFrames  Number of frames to transmit
/// @param[in]  chipSelectID    LEGACY - this parameter is ignored.
/// @retval     >0              Number of transmitted frames
/// @retval     0               Error
DWORD SPIReadWritePolling(DWORD spiChannel, DWORD *rxBuffer, DWORD *txBuffer, DWORD numberOfFrames, DWORD chipSelectID);

//******************************************************************************
/// Writes data to SPI in polling mode
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[in]  buffer          Pointer to the transmit buffer
/// @param[in]  numberOfFrames  Number of frames to transmit
/// @param[in]  chipSelectID    LEGACY - this parameter is ignored.
/// @retval     >0              Number of transmitted frames
/// @retval     0               Error
DWORD SPIWritePolling(DWORD spiChannel, DWORD *buffer, DWORD numberOfFrames, DWORD chipSelectID);

//******************************************************************************
/// Reads data from SPI in polling mode
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[in]  buffer          Pointer to the receive buffer
/// @param[in]  numberOfFrames  Number of frames to receive
/// @param[in]  chipSelectID    LEGACY - this parameter is ignored.
/// @retval                     Number of receiveded frames
DWORD SPIReadPolling(DWORD spiChannel, DWORD *buffer, DWORD numberOfFrames, DWORD chipSelectID);

//******************************************************************************
///
/// Deprecated Defines
///
//******************************************************************************

#define SPI_POLARITY_HIGH  FALSE   ///< SPI Clock Polarity Base High
#define SPI_POLARITY_LOW   TRUE    ///< SPI Clock Polarity Base Low

#define SPI_PHASE_HIGH     FALSE   ///< SPI Clock Edge Active on High Going Edge 
#define SPI_PHASE_LOW      TRUE    ///< SPI Clock Edge Active on Low Going Edge

#define SPI_DMA_ENABLE     TRUE    ///< Enable SPI over DMA

//******************************************************************************
///
/// Deprecated Functions
///
//******************************************************************************

//******************************************************************************
/// Initializes SPI unit
/// @deprecated Please use SPIInit
/// @param[in]  spiChannel      Selected SPI channel (1 to SPI_CHANNELS_MAX)
/// @param[in]  dataBits        Number of Databits [1 to 32]
/// @param[in]  baudRateDivisor For PXA270: Serial bit rate = 13MHz/(baudRateDivisor + 1)\n
///                             (baudRateDivisor Range 0 to 4095) Ignored when Slave\n
///                             For PXA3xx     : the following divisors are valid: 0x00,0x01,0x0F,0x07F are valid.\n
///                             For PXA310 only: set the MSB of the baudRateDivisor to use 26MHz as a base clock, instead of 13MHz.\n
///                             For Tegra2:    : Serial bit rate = 54MHz /((baudRateDivisor/2)+1);  (Range 0-255)
/// @param[in]  isSlave         True=Slave   False=Master
/// @param[in]  polarity        SPI Polarity FALSE: Begin with High
/// @param[in]  phase           SPI Phase FALSE:    Data latched on positive edge
/// @param[in]  dmaEnable       TRUE: dma enabled
/// @retval     TRUE    Success
/// @retval     FALSE   Failure
BOOL InitSPI(DWORD spiChannel, DWORD dataBits, DWORD baudRateDivisor, BOOL isSlave, BOOL polarity, BOOL phase, BOOL dmaEnable);

//******************************************************************************
/// De-Initializes SPI unit.\n
/// Calling this function is very important for releasing resources.\n
/// Otherwise SPI interrupts will be blocked, and can only be re-activated by resetting the system
/// @deprecated Please use SPIDeinit.
/// @param[in]  spiChannel     Selected SPI channel (1 - SPI_CHANNELS_MAX)
void DeinitSPI(DWORD spiChannel);

#ifdef SPIDMADEF

//******************************************************************************
/// Writes data to SPI using DMA.\n
/// The function returns after DMA transfer is finished.\n
/// n.b. uses a packed DWORD interface, tranfers have to be a multiple of 4 bytes  
/// @deprecated Please use SPIReadWrite.
/// @param[in]  spiChannel      Selected SPI channel (1 to SPI_CHANNELS_MAX)
/// @param[in]  buffer          Physical Address of the transmit buffer
/// @param[in]  numberOfBytes   Number of bytes to transmit - must be DWORD aligned
/// @retval     >0              Number of transmitted bytes
/// @retval     0               Error
DWORD WriteSPIDMA(DWORD spiChannel, DWORD *buffer, DWORD numberOfBytes);

//******************************************************************************
/// Reads data from SPI using DMA.\n
/// The function returns after buffer is full or when a timeout occurs.\n
/// n.b. uses a packed DWORD interface, tranfers have to be a multiple of 4 bytes  
/// @deprecated Please use SPIReadWrite.
/// @param[in]  spiChannel      Selected SPI channel (1 to SPI_CHANNELS_MAX)
/// @param[out] buffer          Physical Address of the receive buffer
/// @param[in]  numberOfBytes   Size of the receive buffer in bytes - must be DWORD aligned
/// @param[in]  timeout         If the receive buffer is not full, the function returns after this timeout.\n
///                             Timeout is in milliseconds (0 - SPI_MAX_TIMEOUT)\n
///                             The timeout starts after the first sample is received.\n
///                             The function will not return as long as no samples are received
/// @retval     >0              Number of received bytes
/// @retval     0               Error
DWORD ReadSPIDMA(DWORD spiChannel, DWORD *buffer, DWORD numberOfBytes, DWORD timeout);

//******************************************************************************
/// Simultaneously reads and writes data from/to SPI using DMA.\n
/// The function returns after buffer is full or when a timeout occurs.\n
/// @deprecated Please use SPIReadWrite.
/// @param[in]  spiChannel      Selected SPI channel (1 to SPI_CHANNELS_MAX)
/// @param[out] rxBuffer        Physical Address of the receive buffer
/// @param[in]  txBuffer        Physical Address of the transmit buffer.\n
/// @param[in]  numberOfBytes   Size of the transmit and receive buffers in bytes  - must be DWORD aligned for PXA
/// @param[in]  timeout         If the receive buffer is not full, the function returns after this timeout.\n
/// @retval     >0              Number of received bytes
/// @retval     0               Error
DWORD ReadWriteSPIDMA(DWORD spiChannel, DWORD *rxBuffer, DWORD *txBuffer, DWORD numberOfBytes, DWORD timeout);

//******************************************************************************
/// Writes data to SPI using interrupts.
/// @deprecated Please use SPIReadWrite.
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[in]  buffer          Pointer to the transmit buffer
/// @param[in]  numberOfFrames  Number of frames to transmit
/// @retval     >0              Number of transmitted frames
/// @retval     0               Error
DWORD WriteSPIEx(DWORD spiChannel, DWORD *buffer, DWORD numberOfFrames);

//******************************************************************************
/// Reads data from SPI using interrupts.\n
/// The function returns when the buffer is full or when a timeout occurs
/// @deprecated Please use SPIReadWrite.
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[out] buffer          Pointer to the receive buffer
/// @param[in]  numberOfFrames  Number of frames to receive
/// @param[in]  timeout         If the receive buffer is not full, the function returns after this timeout.\n
///                             Timeout is in milliseconds (0 - SPI_MAX_TIMEOUT)\n
///                             The timeout starts after the first sample is received.\n
///                             The function will not return as long as no samples are received.
/// @retval     >0              Number of received frames
/// @retval     0               Error
DWORD ReadSPIEx(DWORD spiChannel, DWORD *buffer, DWORD numberOfFrames, DWORD timeout);

#endif

//******************************************************************************
/// Writes data to SPI in polling mode
/// @deprecated Please use SPIWritePolling.
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[in]  buffer          Pointer to the transmit buffer
/// @param[in]  numberOfFrames  Number of frames to transmit
/// @return                     Number of transmitted frames
DWORD WriteSPI(DWORD spiChannel, DWORD *buffer, DWORD numberOfFrames);

//******************************************************************************
/// Reads data from SPI in polling mode. The function returns after buffer is full
/// @deprecated Please use SPIReadPolling.
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[out] buffer          Pointer to the receive buffer
/// @param[in]  numberOfFrames  Number of frames to receive
/// @return                     Number of received frames
DWORD ReadSPI(DWORD spiChannel, DWORD *buffer, DWORD numberOfFrames);

//******************************************************************************
/// Simultaneously reads and writes data from/to SPI.\n
/// The function returns after buffer is full or when a timeout occurs.\n
/// @deprecated Please use SPIReadWritePolling.
/// @param[in]  spiChannel      Selected SPI channel (1 - SPI_CHANNELS_MAX)
/// @param[out] rxBuffer        Physical Address of the receive buffer
/// @param[in]  txBuffer        Physical Address of the transmit buffer.\n
/// @param[in]  numberOfBytes   Size of the transmit and receive buffers in bytes  - must be DWORD aligned for PXA
/// @param[in]  timeout         If the receive buffer is not full, the function returns after this timeout. 0 = Infinite timeout\n
/// @retval     >0              Number of received bytes
/// @retval     0               Timeout occured
DWORD ReadWriteSPI(DWORD spiChannel, DWORD *rxBuffer, DWORD *txBuffer, DWORD numberOfBytes, DWORD timeout);

#ifdef __cplusplus
}
#endif

#endif _SPILIB_H_
