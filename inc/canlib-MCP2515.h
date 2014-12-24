///
/// @file        canlib-mcp2515.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: kia $
/// $Rev: 2293 $
/// $Date: 2014-05-27 16:40:57 +0200 (Di, 27 Mai 2014) $
/// @brief       Header for MCP2515 CAN controller library 
///
/// @target      Colibri PXAxxx, T20
///

#ifndef _CANLIB_H_
#define _CANLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#define STANDARD    0x00        ///< Use standard (11 bits ID) frame types
#define EXTENDED    0x01        ///< Use extended (29 bits ID) frame types

#define PXA_TYPE     (0)        ///< PXA type
#define TEG_TYPE     (1)        ///< Tegra type
#define UNKNOWN_TYPE (2)        ///< Unknown type

/// Version info
#define CAN_VER_MAJ     1
#define CAN_VER_MIN     7
#define CAN_VER_BUILD   0

#define MSG_DATA_LENGTH     (8)

/// Structure for standard and extended frame types 
typedef struct tCanMessageExt {
    int     frameType;               ///< 0 = Standard Frame or 1 = Extended Frame
    DWORD   id;                      ///< Message identifier (11 or 29 bits)
    BOOL    isRemoteTransmitRequest; ///< FALSE = transmitted message will be a data frame\n  TRUE = Transmitted Message will be a Remote Transmission Request 
    USHORT  length;                  ///< Message length (0-7)
    USHORT  place;                   ///< This field  is not used, the value ignored.
    ULONG   timeStamp;               ///< This field  is not used, the value ignored.
    ULONG   errorStatus;             ///< This field  is not used, the value ignored.
    UCHAR   data[MSG_DATA_LENGTH];   ///< Message data
} canMessageExt;

//******************************************************************************
/// Writes acceptance code and mask register. The mask register is also stored in a local shadow variable.
/// @param[in]    device        Instance number (0..3)
/// @param[in]    description   New acceptance code(32bits).\n
///                             This parameter was copied from the CAN SJA1000 implementation, but the feature is not available for the MCP2515.\n
///                             It has no effect.
/// @param[in]    mask          New acceptance mask(32bits)
/// @retval       TRUE    Success
/// @retval       FALSE   Failed cases
BOOL CANLibMCP2515WriteDescriptor(DWORD device, DWORD description, DWORD mask);

//******************************************************************************
/// @DEPRECATED
/// Reads acceptance code from the local shadow variable (no SPI communication).\n
/// This function was copied from the CAN SJA1000 implementation, but the feature is not available for the MCP2515.
/// @param[in] device      Instance number (0..3)
/// @retval    Acceptance code
INT32 CANLibMCP2515ReadDescriptor(DWORD device);

//******************************************************************************
/// Reads the mask value from the local shadow variable (no SPI communication)
/// @param[in]      device      Instance number (0..3)
/// @retval         Acceptance Mask
INT32 CANLibMCP2515ReadMask(DWORD device);

//******************************************************************************
/// Clear local read and write message buffers and reset the MCP2515.
/// @param[in]   device      Instance number (0..3)
/// @retval      TRUE   Success
BOOL CANLibMCP2515Reset(DWORD device);

//******************************************************************************
/// Set the CAN Baudrate and configure the MCP2515 mode to "Normal Operation"
/// @param[in]    device    Instance number (0..3)
/// @param[in]    baud      New Baud Rate [kbps]\n
///                         Accepted values are: 1000, 800, 500, 250, 125, 100, 50, 20, 10
/// @retval       TRUE      Mode change successful
/// @retval       FALSE     Failure
BOOL CANLibMCP2515SetBaudrate(DWORD device, DWORD baud);

//******************************************************************************

/// Sends a CAN message
/// @param[in]    device      Instance number (0..3)
/// @param[in]    message     CAN message to transmit. See description of canMessageExt in canlib-MCP2515.h for details.
/// @retval       TRUE    Success
/// @retval       FALSE   Failure
BOOL CANLibMCP2515TransmitMessage(DWORD device, canMessageExt *message, DWORD timeout);

//******************************************************************************
/// Receives CAN messages
/// @param[in]  device              Instance number (0..3)
/// @param[out] message             Aray of received CAN messages. See description of canMessageExt in canlib-MCP2515.h for details.
/// @param[in]  numberOfMessages    Number of messages to receive
/// @param[in]  timeout             Maximum time to wait for a Receive interrupt [ms]\n
///                                 The time-out value needs to be a positive number between zero and 0x7FFFFFFF, or the infinite time-out value 0xFFFFFFFF
/// @retval     Number of Received Messages
DWORD CANLibMCP2515ReceiveMessages(DWORD device, canMessageExt *message, DWORD numberOfMessages, DWORD timeout);

//******************************************************************************
/// Initializes CAN Controller
/// @param[in]  device          Instance number (0..3)\n
///                             The instance number defines the SPI channel used to communicate with the CAN controller:\n
///                             On Colibri PXA: 0->SPI1 1->SPI3 2->SPI2 3->SPI4\n
///                             On Colibri T20: 0->SPI4 1->SPI3 2->SPI2 3->SPI4\n
///                             On Colibri T30: 0->SPI1 1->SPI3 2->SPI2 3->SPI4\n
///                             On Apalis  T30: 0->SPI2 1->SPI4
/// @param[in]  interruptGpio   GPIO where the MCP2515's nINT signal is connected to.
/// @param[in]  waterMark       This parameter is not used anymore, the value ignored.
/// @param[in]  bufferSize      Number of CAN Messages the Receive Buffer can hold
/// @param[in]  processorType   This parameter is not used anymore, the value ignored.
/// @retval     0               Success
/// @retval     !=0             error. the value represents internal error codes
INT32 CANLibMCP2515Init(DWORD device, DWORD interruptGpio, DWORD waterMark, DWORD bufferSize, DWORD processorType);

//******************************************************************************
/// Deinitializes MCP2515 controller and related functions
/// @param[in]    device      Instance number (0..3)
void CANLibMCP2515DeInit(DWORD device);

//******************************************************************************
/// Returns CAN status for the particular instance. \n
/// The error status is sticky.
/// Once an error has occured, this function returns the error value, until it is cleared
/// by calling either CANLibMCP2515Init() or CANLibMCP2515Reset().
/// @param[in]  device      Instance number (0..3)
/// @retval     0           No Error        CAN status information
/// @retval     0x20        The MCP2515 triggered an error interrupt (MCP_ERRIF | MCP_WAKIF | MCP_MERRF)
INT32 CANLibMCP2515GetStatus(DWORD device);

//******************************************************************************
/// @internal
/// Prints message information
/// @param[in]    message     Contains message information in structure format
void CANLibMCP2515PrintMessage(canMessageExt *message);

//******************************************************************************
/// Request Library Revision
/// @param[out]   verMaj        Major library version 
/// @param[out]   verMin        Minor library version 
/// @param[out]   buildNumber   Library Build number
void CANLibMCP2515GetLibVersion(DWORD *verMaj, DWORD *verMin, DWORD *buildNumber);

//******************************************************************************
/// @internal
/// Clear all interrupt flags in the MCP2515 CAN controller. 
/// @param[in]  device      Instance number (0..3)
void CANLibMCP2515ResetInterruptFlags(DWORD device);

/// Outputs the regsister values 
/// @param[in]   device      Instance number
void DumpMcp2515(DWORD device);

#ifdef __cplusplus
}
#endif

#endif
