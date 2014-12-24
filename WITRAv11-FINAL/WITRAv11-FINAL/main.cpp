//#################################################################################
// WITRA - Wearable Interface for Teleoperation of Robotic Arms
// Author: Vinícius Bazan Pinto Fernandes
// Company: Universidade de São Paulo - EESC
// Creation date: 23-Dec-2014
// Version: 11 - 3 IMUs, recording datalog, gripper button added
//
// WITRA is a wearable motion capture interface. It's primary application is the 
// teleoperation of robot arms. It can, however, be used with a variery of systems
// and applications. Due to its simplicity and generic string formattin, using a 
// json-like formatting, almost any system that can receive data over TCP/IP can 
// interpret WITRA's data. Hence, WITRA can be used with drones, PC mouse control,
// PowerPoint presentation control, motion capture and analisys on rehabilitation
// therapies, robot teleoperation or even gaming. 
//#################################################################################

#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Winbase.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <winsock2.h>
#include <stdlib.h>

//INTERRUPT
#include "intlib.h"
#include "CoProcLib.h"
#include "MapRegLib.h"
#include "GpioLib.h"
//--------

#pragma comment(lib, "Ws2.lib")



//******************************************************************************
/// Configures COM port for specified baudrate
/// @param[in]    port         COM port
/// @param[in]    baudRate     Baudrate for communication
/// @retval       TRUE         Success
///               FALSE        Failure

/// Set IP address and port number according to server IP address and port number

//#define SERVER_IP_ADDRESS    "192.168.27.1"       // SERVER LAPTOP
#define SERVER_IP_ADDRESS    "192.168.27.50"  // SERVER SCARA

#define PORT_NUMBER          8000
#define BUFFER_SIZE          128

//******************************************************************************
// Global variables

bool stop1 = false;						// stop flag for MainThread
bool offsetFlag = true;					// flag used to offset yaw the first time MainThread is executed

//******  UART  ******
HANDLE portHandle;
HANDLE portHandle2;
HANDLE portHandle3;
DWORD noOfBytesRead = 0;
DWORD noOfBytesRead2 = 0;
DWORD noOfBytesRead3 = 0;
char receiveBuffer[23] = {0};			// buffer for incoming serial data
char receiveBuffer2[23] = {0};
char receiveBuffer3[23] = {0};
std::vector<double> angles;				// vector of floating point numbers corresponding to the data from the IMU

//******  SOCKETS  *******
WSADATA ws;								///< Structure to contain information about the Windows Socket implementation
SOCKET commSocket;
int retVal_S = 0;
char sendBuffer[BUFFER_SIZE] = {0};
char recvBuffer[BUFFER_SIZE] = {0};
struct sockaddr_in serverinfo;

//***** Human Arm Parameters - spherical coordinates *****
double theta1 = 0.0;        // shoulder-elbow link's footprint orientation on horizontal plane
double theta2 = 0.0;		// elbow-wrist link's footprint orientation on horizontal plane	
double theta3 = 0.0;		// wrist-hand link's footprint orientation on horizontal plane
double phi1 = 0.0;			// shoulder-elbow link's vertical inclination 
double phi2 = 0.0;			// elbow-wrist link's vertical inclination
double phi3 = 0.0;			// wrist-hand link's vertical inclination

const double L1 = 0.25;				   // distance from the shoulder to the elbow in meters  
const double L2 = 0.25;				   // distance from the elbow to the wrist in meters
const double L3 = 0.08;				   // distance from the wrist to the center of the hand in meters

// x, y, z position of the center of the hand
double px = 0.0;				
double py = 0.0;
double pz = L1 + L2 + L3;
//*******************************

//******** Hysteresys Loop *******
double difTheta1 = 0.0;
double difTheta2 = 0.0;
double difTheta3 = 0.0;
double difPhi1 = 0.0;
double difPhi2 = 0.0;
double difPhi3 = 0.0;
double lastTheta1 = 0.0;
double lastTheta2 = 0.0;
double lastTheta3 = 0.0;
double lastPhi1 = 0.0;
double lastPhi2 = 0.0;
double lastPhi3 = 0.0;

double tolerance = 0.05; // degrees
//*******************************

//********** Datalog *************
std::vector<double> log_px;
std::vector<double> log_py;
std::vector<double> log_pz;
std::vector<int> log_g;
//********************************

//******** INTERRUPT - GRIPPER BUTTON *******
    PIN_INSTANCE gpio;
    HANDLE hEvent = NULL;  
    BOOL pinLevel = FALSE;
    BOOL returnFlag = FALSE;
    DWORD irq = 0;   
    DWORD gpioNumber = 0;
    DWORD interruptEdge = 0;
    DWORD systemInterrupt = 0;
	BOOL gripperFlag = FALSE;
//*******************************************

// MainThread() loop count
int i = 1;

//*******************************************************************************************************
// function to create and open COM3
BOOL PortOpen(HANDLE *port, DWORD baudRate)
{
    DCB portDCB;                                              ///< COM port configuration structure
    BOOL returnValue = FALSE;
    COMMTIMEOUTS comTimeOut;
	baudRate = strtol("57600", 0,0);
   
	/// Open interface to reader
    *port = CreateFile(TEXT("COM3:"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
    if (*port == INVALID_HANDLE_VALUE)
    {
        printf("Error Opening COM Port 1\n");
        return FALSE;
    }
 
    /// COM Port Configuration
    portDCB.DCBlength = sizeof (DCB);                         ///< Initialize the DCBlength member
    GetCommState (*port, &portDCB);                           ///< Get the default port setting information.
    /// Change the DCB structure settings
    portDCB.BaudRate = baudRate;                              ///< Current baud 
    portDCB.fBinary = TRUE;                                   ///< Binary mode; no EOF check 
    portDCB.fParity = FALSE;                                  ///< Disable parity checking 
    portDCB.fOutxCtsFlow = FALSE;                             ///< No CTS output flow control 
    portDCB.fOutxDsrFlow = FALSE;                             ///< No DSR output flow control 
    portDCB.fDtrControl = DTR_CONTROL_DISABLE;                ///< Disable DTR flow control type 
    portDCB.fDsrSensitivity = FALSE;                          ///< DSR sensitivity 
    portDCB.fTXContinueOnXoff = TRUE;                         ///< XOFF continues Tx 
    portDCB.fOutX = FALSE;                                    ///< No XON/XOFF out flow control 
    portDCB.fInX = FALSE;                                     ///< No XON/XOFF in flow control 
    portDCB.fErrorChar = FALSE;                               ///< Disable error replacement 
    portDCB.fNull = FALSE;                                    ///< Disable null stripping 
    portDCB.fRtsControl = RTS_CONTROL_DISABLE;                ///< Disable RTS flow control 
    portDCB.fAbortOnError = FALSE;                            ///< Do not abort reads/writes on error
    portDCB.ByteSize = 8;                                     ///< Number of bits/byte, 4-8 
    portDCB.Parity = NOPARITY;                                ///< 0-4 = no, odd, even, mark, space 
    portDCB.StopBits = ONESTOPBIT;                            ///< 0, 1, 2 = 1, 1.5, 2 
 
    /// Configure the port according to the specifications of the DCB structure
    if (!SetCommState (*port, &portDCB))
    {
      printf("Error Configuring COM Port 1\n");                 ///< Could not configure the serial port
      return FALSE;
    }
 
    /// Get communication time out values
    returnValue = GetCommTimeouts(*port, &comTimeOut);
    comTimeOut.ReadIntervalTimeout = 10;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    /// Set communication time out values
    returnValue = SetCommTimeouts(*port, &comTimeOut);
 
    return TRUE;
}

// function to create and open COM2
BOOL PortOpen2(HANDLE *port, DWORD baudRate)
{
    DCB portDCB;                                              ///< COM port configuration structure
    BOOL returnValue = FALSE;
    COMMTIMEOUTS comTimeOut;
	baudRate = strtol("57600", 0,0);
    
	/// Open interface to reader
    *port = CreateFile(TEXT("COM2:"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
    if (*port == INVALID_HANDLE_VALUE)
    {
        printf("Error Opening COM Port 2\n");
        return FALSE;
    }
 
    /// COM Port Configuration
    portDCB.DCBlength = sizeof (DCB);                         ///< Initialize the DCBlength member
    GetCommState (*port, &portDCB);                           ///< Get the default port setting information.
    /// Change the DCB structure settings
    portDCB.BaudRate = baudRate;                              ///< Current baud 
    portDCB.fBinary = TRUE;                                   ///< Binary mode; no EOF check 
    portDCB.fParity = FALSE;                                  ///< Disable parity checking 
    portDCB.fOutxCtsFlow = FALSE;                             ///< No CTS output flow control 
    portDCB.fOutxDsrFlow = FALSE;                             ///< No DSR output flow control 
    portDCB.fDtrControl = DTR_CONTROL_DISABLE;                ///< Disable DTR flow control type 
    portDCB.fDsrSensitivity = FALSE;                          ///< DSR sensitivity 
    portDCB.fTXContinueOnXoff = TRUE;                         ///< XOFF continues Tx 
    portDCB.fOutX = FALSE;                                    ///< No XON/XOFF out flow control 
    portDCB.fInX = FALSE;                                     ///< No XON/XOFF in flow control 
    portDCB.fErrorChar = FALSE;                               ///< Disable error replacement 
    portDCB.fNull = FALSE;                                    ///< Disable null stripping 
    portDCB.fRtsControl = RTS_CONTROL_DISABLE;                ///< Disable RTS flow control 
    portDCB.fAbortOnError = FALSE;                            ///< Do not abort reads/writes on error
    portDCB.ByteSize = 8;                                     ///< Number of bits/byte, 4-8 
    portDCB.Parity = NOPARITY;                                ///< 0-4 = no, odd, even, mark, space 
    portDCB.StopBits = ONESTOPBIT;                            ///< 0, 1, 2 = 1, 1.5, 2 
 
    /// Configure the port according to the specifications of the DCB structure
    if (!SetCommState (*port, &portDCB))
    {
      printf("Error Configuring COM Port 2\n");                 ///< Could not configure the serial port
      return FALSE;
    }
 
    /// Get communication time out values
    returnValue = GetCommTimeouts(*port, &comTimeOut);
    comTimeOut.ReadIntervalTimeout = 10;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    /// Set communication time out values
    returnValue = SetCommTimeouts(*port, &comTimeOut);
 
    return TRUE;
}

// function to create and open COM1
BOOL PortOpen3(HANDLE *port, DWORD baudRate)
{
    DCB portDCB;                                              ///< COM port configuration structure
    BOOL returnValue = FALSE;
    COMMTIMEOUTS comTimeOut;
	baudRate = strtol("57600", 0,0);
    
	/// Open interface to reader
    *port = CreateFile(TEXT("COM1:"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
    if (*port == INVALID_HANDLE_VALUE)
    {
        printf("Error Opening COM Port 3\n");
        return FALSE;
    }
 
    /// COM Port Configuration
    portDCB.DCBlength = sizeof (DCB);                         ///< Initialize the DCBlength member
    GetCommState (*port, &portDCB);                           ///< Get the default port setting information.
    /// Change the DCB structure settings
    portDCB.BaudRate = baudRate;                              ///< Current baud 
    portDCB.fBinary = TRUE;                                   ///< Binary mode; no EOF check 
    portDCB.fParity = FALSE;                                  ///< Disable parity checking 
    portDCB.fOutxCtsFlow = FALSE;                             ///< No CTS output flow control 
    portDCB.fOutxDsrFlow = FALSE;                             ///< No DSR output flow control 
    portDCB.fDtrControl = DTR_CONTROL_DISABLE;                ///< Disable DTR flow control type 
    portDCB.fDsrSensitivity = FALSE;                          ///< DSR sensitivity 
    portDCB.fTXContinueOnXoff = TRUE;                         ///< XOFF continues Tx 
    portDCB.fOutX = FALSE;                                    ///< No XON/XOFF out flow control 
    portDCB.fInX = FALSE;                                     ///< No XON/XOFF in flow control 
    portDCB.fErrorChar = FALSE;                               ///< Disable error replacement 
    portDCB.fNull = FALSE;                                    ///< Disable null stripping 
    portDCB.fRtsControl = RTS_CONTROL_DISABLE;                ///< Disable RTS flow control 
    portDCB.fAbortOnError = FALSE;                            ///< Do not abort reads/writes on error
    portDCB.ByteSize = 8;                                     ///< Number of bits/byte, 4-8 
    portDCB.Parity = NOPARITY;                                ///< 0-4 = no, odd, even, mark, space 
    portDCB.StopBits = ONESTOPBIT;                            ///< 0, 1, 2 = 1, 1.5, 2 
 
    /// Configure the port according to the specifications of the DCB structure
    if (!SetCommState (*port, &portDCB))
    {
      printf("Error Configuring COM Port 3\n");                 ///< Could not configure the serial port
      return FALSE;
    }
 
    /// Get communication time out values
    returnValue = GetCommTimeouts(*port, &comTimeOut);
    comTimeOut.ReadIntervalTimeout = 10;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    /// Set communication time out values
    returnValue = SetCommTimeouts(*port, &comTimeOut);
 
    return TRUE;
}

//******************************************************************************
/// Close UART port
/// @param[in]    port     COM port
/// @retval       TRUE     Success
///               FALSE    Failure
BOOL PortClose(HANDLE *port)
{
    if (*port == NULL)
    {
        return FALSE;
    }
    CloseHandle(*port);
    *port = NULL;
    return TRUE;
}
 
//**************************************************************************************************
// Main Thread - deals with serial data, kinematics and socket data sending
DWORD WINAPI MainThread(LPVOID pParam)
	{
		 		
		//***** Variables for Thread Timing *****
		DWORD dwOldTime;
		DWORD dwTimeElapsed;
		//********************************

		//***** Variables for string manipulation *****
		std::string delimiter1 = " ";          // delimiter to split the string of Y,P,R
		std::string delimiter2 = "\r";         // delimiter to split the end of the string of Y,P,R

		std::string str;                       // receiveBuffer is converted into this string
		size_t pos = 0;                        // position of the string for using substrings
		std::string token;                     // token (part) of a string
		double temp;                           // temporary variable to store the convertion from string to float (of Y,P,R)
		
		std::string str2;                      // receiveBuffer is converted into this string
		size_t pos2 = 0;                       // position of the string for using substrings
		std::string token2;                    // token (part) of a string
		double temp2;                          // temporary variable to store the convertion from string to float (of Y,P,R)

		std::string str3;                      // receiveBuffer is converted into this string
		size_t pos3 = 0;                       // position of the string for using substrings
		std::string token3;                    // token (part) of a string
		double temp3;                          // temporary variable to store the convertion from string to float (of Y,P,R)
		//*********************************************

		// initialization of the angles vector
		angles.push_back(0.0);  // IMU 1 YAW
		angles.push_back(0.0);  // IMU 1 PITCH
		angles.push_back(0.0);  // IMU 1 ROLL
		angles.push_back(0.0);  // IMU 2 YAW
		angles.push_back(0.0);  // IMU 2 PITCH
		angles.push_back(0.0);  // IMU 2 ROLL
		angles.push_back(0.0);  // IMU 3 YAW
		angles.push_back(0.0);  // IMU 3 PITCH
		angles.push_back(0.0);  // IMU 3 ROLL

		// offsets
		double yawOffset;
		double yawOffset2;
		double yawOffset3;
		double pitchOffset;
		double pitchOffset2;
		double pitchOffset3;
		double rollOffset;
		double rollOffset2;
		double rollOffset3; 

		// Intermediate kinematic variables - shoulder position = (0,0,0)
		// position of the elbow
		double x1 = 0.0;
		double y1 = 0.0;
		double z1 = 0.0;
		// position of the wrist
		double x2 = 0.0;
		double y2 = 0.0;
		double z2 = 0.0;
		

                while( stop1 == false ) // while ENTER is not pressed, continue the program execution
                {						
					dwOldTime = GetTickCount();    // starts time counter
                   
					// ************* Reads data on serial port ****************************
					ReadFile(portHandle, receiveBuffer, 23, &noOfBytesRead, NULL);
					ReadFile(portHandle2, receiveBuffer2, 23, &noOfBytesRead2, NULL);
					ReadFile(portHandle3, receiveBuffer3, 23, &noOfBytesRead3, NULL);
					
					str = std::string(receiveBuffer);           // converts receiveBuffer (a char[]) to string
					str2 = std::string(receiveBuffer2);         // converts receiveBuffer2 (a char[]) to string
					str3 = std::string(receiveBuffer3);         // converts receiveBuffer3 (a char[]) to string
					// ********************************************************************

					// ************ String handling - separates the variables *************
					
					pos = str.find(delimiter1);                 // finds the position of the first white space on the string
					token = str.substr(0, pos);                 // parses the string at this position
					std::istringstream(token) >> temp;          // passes a string (token) into a stream and stores in temp (float) - casting
					angles[0] = temp;                           // angles[0] = IMU 1 Yaw;
					
					pos2 = str2.find(delimiter1);               // finds the position of the first white space on the string
					token2 = str2.substr(0, pos2);              // parses the string at this position
					std::istringstream(token2) >> temp2;        // passes a string (token) into a stream and stores in temp (float) - casting
					angles[3] = temp2;                          // angles[3] = IMU 2 Yaw

					pos3 = str3.find(delimiter1);               // finds the position of the first white space on the string
					token3 = str3.substr(0, pos3);              // parses the string at this position
					std::istringstream(token3) >> temp3;        // passes a string (token) into a stream and stores in temp (float) - casting
					angles[6] = temp3;                          // angles[6] = IMU 3 Yaw
					
					str.erase(0, pos + delimiter1.length());    // erases the part of the string that was already read (i.e. yaw value plus the white space)
					str2.erase(0, pos2 + delimiter1.length());  // erases the part of the string that was already read (i.e. yaw value plus the white space)
					str3.erase(0, pos3 + delimiter1.length());  // erases the part of the string that was already read (i.e. yaw value plus the white space)
					

					pos = str.find(delimiter1);                 // finds the second white space
					token = str.substr(0, pos);
					std::istringstream(token) >> temp;
					angles[1] = temp;                           // angles[1] = IMU 1 Pitch;

					pos2 = str2.find(delimiter1);               // finds the second white space
					token2 = str2.substr(0, pos2);
					std::istringstream(token2) >> temp2;
					angles[4] = temp2;                           // angles[4] = IMU 2 Pitch;

					pos3 = str3.find(delimiter1);               // finds the second white space
					token3 = str3.substr(0, pos3);
					std::istringstream(token3) >> temp3;
					angles[7] = temp3;                           // angles[7] = IMU 3 Pitch;

					
					str.erase(0, pos + delimiter1.length());
					str2.erase(0, pos2 + delimiter1.length());
					str3.erase(0, pos3 + delimiter1.length());

					pos = str.find(delimiter2);                // finds the end of the string (\r)
					token = str.substr(0, pos);
					std::istringstream(token) >> temp;
					angles[2] = temp;                          // angles[2] = IMU 1 Roll

					pos2 = str2.find(delimiter2);                // finds the end of the string (\r)
					token2 = str2.substr(0, pos2);
					std::istringstream(token2) >> temp2;
					angles[5] = temp2;                          // angles[5] = IMU 2 Roll

					pos3 = str3.find(delimiter2);                // finds the end of the string (\r)
					token3 = str3.substr(0, pos3);
					std::istringstream(token3) >> temp3;
					angles[8] = temp3;                          // angles[8] = IMU 3 Roll
					// ******************************************************************************


					// ************************ Offsets ************************************
			
					// Sets a offset for Yaw. That is, when the program starts, it will set Yaw to zero
					if (offsetFlag == true){    // does on the first time. Down we change the flag to false
						yawOffset = angles[0];
						pitchOffset = angles[1];
						rollOffset = angles[2];
						
						yawOffset2 = angles[3];
						pitchOffset2 = angles[4];
						rollOffset2 = angles[5];

						yawOffset3 = angles[6];
						pitchOffset3 = angles[7];
						rollOffset3 = angles[8];						
					}
					
					// If, in initialization, yaw is in the second quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the fourth quadrant to avoid crossing it
					if (yawOffset >= 90.0 && yawOffset <= 180.0){
						if (angles[0] < 0.0){
							angles[0] = 360.0 + angles[0];
						}
					}

					// If, in initialization, yaw is in the third quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the first quadrant to avoid crossing it
					if (yawOffset >= -180.0 && yawOffset <= -90.0){
						if (angles[0] > 0) {
							angles[0] = angles[0] - 360.0;
						}
					}

					// If, in initialization, yaw is in the second quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the fourth quadrant to avoid crossing it
					if (yawOffset2 >= 90.0 && yawOffset2 <= 180.0){
						if (angles[3] < 0.0){
							angles[3] = 360.0 + angles[3];
						}
					}

					// If, in initialization, yaw is in the third quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the first quadrant to avoid crossing it
					if (yawOffset2 >= -180 && yawOffset2 <= -90){
						if (angles[3] > 0) {
							angles[3] = angles[3] - 360;
						}
					}

					// If, in initialization, yaw is in the second quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the fourth quadrant to avoid crossing it
					if (yawOffset3 >= 90.0 && yawOffset3 <= 180.0){
						if (angles[6] < 0.0){
							angles[6] = 360.0 + angles[6];
						}
					}

					// If, in initialization, yaw is in the third quadrant, it may cross the 180/-180 border
					  // this removes this border and sets it in the first quadrant to avoid crossing it
					if (yawOffset3 >= -180 && yawOffset3 <= -90){
						if (angles[6] > 0) {
							angles[6] = angles[6] - 360;
						}
					}

					// sets the offsets
					angles[0] = angles[0] - yawOffset;           // yaw starts at 0 degrees
					angles[1] = angles[1] - pitchOffset;         // pitch starts at 0 degrees
					angles[2] = angles[2] - rollOffset;			 // roll starts at 0 degrees
					angles[3] = angles[3] - yawOffset2;     
					angles[4] = angles[4] - pitchOffset2;
					angles[5] = angles[5] - rollOffset2;
					angles[6] = angles[6] - yawOffset3;     
					angles[7] = angles[7] - pitchOffset3;
					angles[8] = angles[8] - rollOffset3;
					// ***************************************************************************


					//**************** Calculates the Direct Kinematics of the Human Arm *******************

					theta1 = angles[0];
					phi1 = -angles[1];  // negative signal because when the arm is moved up, the reading is negative
					theta2 = angles[3];
					phi2 = -angles[4];  // negative signal because when the arm is moved up, the reading is negative
					theta3 = angles[6];
					phi3 = -angles[7];  // negative signal because when the arm is moved up, the reading is negative


					// Hysteresys loop - if the difference between the actual and previous values 
					// is less or equal to the tolerance, keeps the previous value. Noise is removed
					if (offsetFlag == false){
						difTheta1 = theta1 - lastTheta1;
						difTheta2 = theta2 - lastTheta2;
						difTheta3 = theta3 - lastTheta3;
						difPhi1 = phi1 - lastPhi1;
						difPhi2 = phi2 - lastPhi2;
						difPhi3 = phi3 - lastPhi3;
												
						if (abs(difTheta1) <= tolerance){ 
							theta1 = lastTheta1;
						}
						else {
							lastTheta1 = theta1;
						}

						if (abs(difTheta2) <= tolerance){  
							theta2 = lastTheta2;
						}
						else {
							lastTheta2 = theta2;
						}

						if (abs(difTheta3) <= tolerance){  
							theta3 = lastTheta3;
						}
						else {
							lastTheta3 = theta3;
						}

						if (abs(difPhi1) <= tolerance){  
							phi1 = lastPhi1;
						}
						else {
							lastPhi1 = phi1;
						}

						if (abs(difPhi2) <= tolerance){  
							phi2 = lastPhi2;
						}
						else {
							lastPhi2 = phi2;
						}

						if (abs(difPhi3) <= tolerance){ 
							phi3 = lastPhi3;
						}
						else {
							lastPhi3 = phi3;
						}
					}
					else {
						lastTheta1 = angles[0];
						lastTheta2 = angles[3];
						lastTheta3 = angles[6];
						lastPhi1 = -angles[1];
						lastPhi2 = -angles[4];
						lastPhi3 = -angles[7];

						offsetFlag = false;
					}

					theta1 = theta1*3.14159/180.0;
					phi1 = phi1*3.14159/180.0; 
					theta2 = theta2*3.14159/180.0;
					phi2 = phi2*3.14159/180.0; 
					theta3 = theta3*3.14159/180.0;
					phi3 = phi3*3.14159/180.0; 

					// Truncates the values to 2 decimal digits
					theta1 = std::floor(100 * theta1) / 100;
					phi1 = std::floor(100 * phi1) / 100;
					theta2 = std::floor(100 * theta2) / 100;
					phi2 = std::floor(100 * phi2) / 100;
					theta3 = std::floor(100 * theta3) / 100;
					phi3 = std::floor(100 * phi3) / 100;
 
					x1 = L1*sin(phi1);
					y1 = L1*cos(phi1)*sin(theta1);
					z1 = L1*cos(phi1)*cos(theta1);

					x2 = L2*sin(phi2) + x1;
					y2 = L2*cos(phi2)*sin(theta2) + y1;
					z2 = L2*cos(phi2)*cos(theta2) + z1;

					px = L3*sin(phi3) + x2;
					py = L3*cos(phi3)*sin(theta3) + y2;
					pz = L3*cos(phi3)*cos(theta3) + z2;
						
					double tempx = px;
					double tempy = py;
					double tempz = pz;

					// mapping from user's reachable space to the robot's one
					py = 0.8*tempz + 0.16;
					px = 0.8*tempy + 0.16;
					pz = tempx/3.0;

					// for safety, the robot's z-coordinate cannot be greater than zero
					if (pz > 0.0){
						pz = 0.0;
					}
					
					// saves the data on the log vectors
					log_px.push_back(px);
					log_py.push_back(py);
					log_pz.push_back(pz);
					log_g.push_back(gripperFlag);

					// formats the string that will be sent over sockets
					sprintf(sendBuffer,"{\"Command\":\"Impedance\",\"X\":%.4lf,\"Y\":%.4lf,\"Z\":%.4lf,\"Gripper\":%d}",px,py,pz,gripperFlag);

					retVal_S = send(commSocket, sendBuffer, BUFFER_SIZE, 0);
					if (retVal_S == SOCKET_ERROR)
					{
						printf("\nCould not send message to Server with error code : %d", WSAGetLastError());
					}
					//*********************************************************************
					
					dwTimeElapsed = GetTickCount();             // gets the final thread time
					dwTimeElapsed = dwTimeElapsed - dwOldTime;  // thread execution time
					
					// Prints Thead execution time
					std::cout << "Thread Time: " << dwTimeElapsed << std::endl << std::endl;
					//i++;
					PurgeComm(portHandle, PURGE_RXCLEAR);        ///< Clear receive buffer
					PurgeComm(portHandle2, PURGE_RXCLEAR);
					PurgeComm(portHandle3, PURGE_RXCLEAR);
					Sleep(20);
                }
                return 0;  
	}

// **************************************************************************************************************
// Gripper thread - waits for interrupt on button. When that happens, the gripper status (open/closed) is toogled

DWORD WINAPI GripperThread(LPVOID pParam)
{
	while( stop1 == false )
    {
		if (WaitForSingleObject(hEvent, INFINITE) == WAIT_OBJECT_0)
        {
			gripperFlag = !gripperFlag;
            if (gripperFlag == TRUE)
            {
                printf("CLOSE\n");
            }
            else 
            {
                printf("OPEN\n");
            }
            InterruptDoneCompat(systemInterrupt);
			Sleep(100);
        }
	}
	return 0;
}

//******************************************************************************
// MAIN
int wmain(void)
{

    DWORD firstChoice = 0;
    BOOL retVal = FALSE;
	BOOL retVal2 = FALSE;
	BOOL retVal3 = FALSE;
	DWORD ThreadID;
	DWORD GripperThreadID;
	HANDLE hMainThread;
	HANDLE hGripperThread;
	DWORD StopID;
	
	//************************  SOCKET CONFIGURATION  ******************************
	retVal_S = WSAStartup(0x0101, &ws);                              ///< Initialize ws2.dll (library used for socket programming)
    if (retVal_S != 0)                                               ///< If WSAStartup failed to initialize
    {
        printf("WSAStartup failed with error: %d\n", WSAGetLastError());
        exit(1);
    }
 
    commSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);        ///< Socket creation
    if (commSocket == INVALID_SOCKET)
    {
        printf("\nSocket creation failed with error code : %d", WSAGetLastError());
        WSACleanup();
        exit(1);
    }
 
    /// Socket binding
    serverinfo.sin_family = AF_INET;                                ///< TCP/UDP socket
    serverinfo.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS);      ///< IP Address of Server
    serverinfo.sin_port = htons(PORT_NUMBER);                       ///< Port number used for communication
 
    retVal_S = connect(commSocket, (LPSOCKADDR)&serverinfo, sizeof(struct sockaddr));   ///< Connect to Server
    if (retVal_S == SOCKET_ERROR)
    {
        printf("\nCould not connect to Server with error code : %d", WSAGetLastError()); 
        WSACleanup();
		getchar();
        return FALSE;
    }
 
	//**************************  UART CONFIGURATION  ***************************	
    retVal = PortOpen(&portHandle, 57600);
	
	retVal2 = PortOpen2(&portHandle2, 57600);

	retVal3 = PortOpen3(&portHandle3, 57600);

    if (!retVal)
    {
        printf("Could not open COM port 1");
        getchar();
        return FALSE;
    }   
	else if (!retVal2)
    {
        printf("Could not open COM port 2");
        getchar();
        return FALSE;
    } 
	else if (!retVal3){
		printf("Could not open COM port 3");
        getchar();
        return FALSE;
	}
    else
    {
        retVal = FALSE;
		retVal2 = FALSE;
		retVal3 = FALSE;
       
            memset(receiveBuffer, 0, 24); 

			memset(receiveBuffer2, 0, 24);

			memset(receiveBuffer3, 0, 24);

	// ********************* GRIPPER ************************

	SetPinAltFn(133, -1, DIR_IN);                      ///< Set SODIMM pin 133 as GPIO input to capture interrupt
	GetGPIOFromPin(133, FALSE, &gpio);                 ///< Get GPIO number for SODIMM pin 133
    gpioNumber = gpio.inst1;

    irq = GetGPIOIrq(gpioNumber);                      ///< Request interrupt (IRQ) on GPIO number
    if (!irq)
    {
        printf("cannot obtain IRQ for GPIO Number %d", gpioNumber);
        getchar();
        return -1;
    }

    interruptEdge = GPIO_EDGE_RISING | GPIO_EDGE_FALLING;                   ///< Configure external interrupt signal as rising edge
    if (!SetGPIOIrqEdge(gpioNumber, interruptEdge)) 
    {
        printf("cannot set GPIO interrupt edge detect");
        getchar();
        return -1;
    }
    hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);     ///< Create event or check for existing event to wait for
    if (!hEvent) 
    {
        printf("Event cannot be created");
        getchar();
        return -1;
    }
    systemInterrupt = RequestSysInterrupt(irq);         ///< Get the system interrupt number for corresponding irq
    if (!systemInterrupt)
    {
        printf("cannot obtain system interrupt number");
        getchar();
        return -1;
    }
 
    if (!InterruptInitializeCompat(systemInterrupt, hEvent, NULL, 0))     ///<  Initialize system interrupt
    {
        printf("cannot Initialize interrupt");
        ReleaseSysIntr(systemInterrupt);
        getchar();
        return -1;
    }

            printf("\n\n\n*****************************************\n");
			printf("Welcome to WITRA: Wearable Interface for Teleoperation of Robot Arms\n");
            printf("*****************************************\n\n");
			

			// Creates the Main Thread 
			hMainThread = CreateThread( NULL,
										   0,
										   MainThread,
										   NULL, 
										   0,
										   &ThreadID );
			
			hGripperThread = CreateThread( NULL,
										   0,
										   GripperThread,
										   NULL, 
										   0,
										   &GripperThreadID );
			

        
					getchar();   // when ENTER is pressed, the execution stops and comes to this point
					stop1 = true;


						// Save data to datalog
						std::ofstream logx;
						std::ofstream logy;
						std::ofstream logz;
						std::ofstream logg;

						logx.open("px.txt");
						for (size_t i = 0; i < log_px.size(); i++){
							logx << log_px[i] << std::endl;
						}
						logx.close();

						logy.open("py.txt");
						for (size_t i = 0; i < log_py.size(); i++){
							logy << log_py[i] << std::endl;
						}
						logy.close();
						
						logz.open("pz.txt");
						for (size_t i = 0; i < log_pz.size(); i++){
							logz << log_pz[i] << std::endl;
						}
						logz.close();

						logg.open("gripper.txt");
						for (size_t i = 0; i < log_g.size(); i++){
							logg << log_g[i] << std::endl;
						}
						logg.close();

						std::cout << "Log saved" << std::endl;
				


					CloseHandle(hMainThread);
					CloseHandle(hGripperThread);

					getchar();  // when ENTER is pressed a second time, the program is finished

					closesocket(commSocket);
					WSACleanup();  

					InterruptDisableCompat(systemInterrupt);                  ///< Disable interrupt
					if (!ReleaseSysIntr(systemInterrupt)) 
					{
						MessageBox(NULL, L"An Error occurred!\ninterrupt was not released correctly", L"ERROR", MB_OK);
						return -1;
					}
					
					return 0;
    }
}
 