///
/// @file        CanLib.h
/// @copyright   Copyright (c) 2013 Toradex AG
/// $Author: kia $
/// $Rev: 1819 $ 
/// $Date: 2013-09-23 13:46:32 +0200 (Mo, 23 Sep 2013) $
/// @brief       Library to interface with a SJA1000 Stand-alone CAN Controller
///
/// @target      PXAxxx & Txx
///

#ifndef _CANLIB_H_
#define _CANLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#define STANDARD    0x00        /// Use standard (11 bit id) frame types
#define EXTENDED    0x01        /// Use extended (29 bit id) frame types

/// Version Info
#define CAN_VER_MAJ     3
#define CAN_VER_MIN     3
#define CAN_VER_BUILD   2

/// Structure for extended frame types (29 Bit adress-range).
/// Set "frametype" to "STANDARD" or "EXTENDED".
/// The RTR (Remote-Frame) flag and data length code are now in seperate
/// variables for better hardware independence !
typedef struct _CAN_MESSAGE_EXT {
    int     frametype;          /// STANDARD or EXTENDED
    DWORD   id;                 /// Message Identifier (11 or 29 Bit)
    BOOL    rtr;                /// True: Remote Frame (ignore length)
    USHORT  length;             /// Message length (0-7)
    USHORT  place;              /// Message place
    ULONG   time_stamp;         /// (0 if not supported)
    ULONG   ErrorStatus;        /// != 0  -> this is an error frame
    UCHAR   data[8];            /// Message data
} CAN_MESSAGE_EXT, *PCAN_MESSAGE_EXT;

//******************************************************************************
/// Returns the library version.
/// @param[out]    VersionMajor   Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]    VersionMinor   Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]    Build          Returns the build number.         Set this parameter to NULL if not required.
/// @retval        NA
void CANGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Initialise CAN Controller
/// @param[in]    device         CAN Device ID      
/// @param[in]    base           SJA1000 Chip Base Address
/// @param[in]    interruptGPIO  GPIO Number of the interrupt from the SJA1000
/// @param[in]    waterMark      Number of messages received before the library signals reception of messages.
/// @param[in]    bufferSize     Size of the receive buffer in CAN_MESSAGE_EXT messages
/// @retval       0 = Success\n
///               1 = SoC not supported\n
///               -1 = Could not create ISR_COM struct\n
///               -2 = Could not map Chip Base Address\n
///               -4 = Could not create Mutex\n
///               -5 = Could not enter reset mode\n
///               -6 = Could not enter normal mode\n
///               -7 = Could not create receive buffer\n
///               -8 = Could not create Interrupt Service Thread\n
///               -9 = Could not initialise GPIO
DWORD InitSJA1000(DWORD device,DWORD base, DWORD interruptGPIO, DWORD waterMark, DWORD bufferSize);

//******************************************************************************
/// Release CAN Controller
/// @param[in]    device         CAN Device ID      
/// @retval       NA
void DeInitSJA1000(DWORD device);

//******************************************************************************
/// Reinitialise the CAN Controller/n
/// Clears any unserviced interrupts.
/// @param[in]    device         CAN Device ID      
/// @retval       NA
void CANReInitialise(DWORD device);

//******************************************************************************
/// Write the Acceptance Code and Mask Registers./n
/// Library is running in dual mode (AFM =0)./n
/// For more information see the SJA1000 documentation.
/// @param[in]    device    CAN Device ID      
/// @param[in]    descr     32 bit acceptance code      
/// @param[in]    mask      32 bit acceptance mask      
/// @retval       TRUE: Success, FALSE: Error
BOOL CanWriteDescriptor(DWORD device, DWORD descr, DWORD mask);

//******************************************************************************
/// Read the Acceptance Code
/// @param[in]    device    CAN Device ID       
/// @retval       Acceptance Code
DWORD CanReadDescriptor(DWORD device);

//******************************************************************************
/// Read the Mask
/// @param[in]    device    CAN Device ID       
/// @retval       Mask
DWORD CanReadMask(DWORD device);

//******************************************************************************
/// Reset the CanChip and clear all buffers
/// @param[in]    device    CAN Device ID      
/// @retval       TRUE: Success, FALSE: Error
BOOL CanReset(DWORD device);

//******************************************************************************
/// Set the Baudrate./n
/// 1000, 800, 500, 250, 125, 100, 50, 20, 10 kBaud./n
/// All other settings set the baud to 125 kBaud
/// @param[in]    device    CAN Device ID      
/// @param[in]    baud      Baudrate
/// @retval       TRUE: Success, FALSE: Error
BOOL CanSetBaudrate(DWORD device, DWORD baud);

//******************************************************************************
/// Send a CAN Message
/// @param[in]    device    CAN Device ID      
/// @param[in]    msg       Pointer to a CAN message structure
/// @retval       TRUE: Success, FALSE: Error
BOOL CanTransmitMessage(DWORD device, CAN_MESSAGE_EXT *msg);

//******************************************************************************
/// Receive CAN messages
/// @param[in]    device         CAN Device ID      
/// @param[out]   msg            pointer to CAN_MESSAGE_EXT array sufficient to hold numberOfMsg messages
/// @param[in]    numberOfMsg    Number of messages to wait for
/// @param[in]    timeout        Timeout value in milliseconds, use INFINITE to wait forever
/// @retval       Number messages received, 0 = timeout
DWORD CANReceiveMessages(DWORD device, CAN_MESSAGE_EXT *msg, DWORD numberOfMsg, DWORD timeout);

//******************************************************************************
/// Return the CAN Status
/// @param[in]    device         CAN Device ID      
/// @retval       0 = OK/n
///               Test with CAN_ERROR and CAN_OVERRUN_HW Constants
DWORD GetCanStatus(DWORD device);

//******************************************************************************
/// Print the contents of the supplied CAN_MESSAGE_EXT message
/// @param[in]    msg            Pointer to a CAN_MESSAGE_EXT instance
/// @retval       NA
void PrintMessage(CAN_MESSAGE_EXT *msg);

#ifdef __cplusplus
}
#endif

#endif