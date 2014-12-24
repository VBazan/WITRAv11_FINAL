//#################################################################################
// WITRA - Wearable Interface for Teleoperation of a Robotic Arm
// Author: Vinícius Bazan Pinto Fernandes
// Company: Universidade de São Paulo - EESC
// Creation date: 11-Dec-2014
// Version: REPEAT TELEOPERATION MOVEMENTS
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
// 
#include <stdlib.h>



#pragma comment(lib, "Ws2.lib")



//******************************************************************************
/// Configures COM port for specified baudrate
/// @param[in]    port         COM port
/// @param[in]    baudRate     Baudrate for communication
/// @retval       TRUE         Success
///               FALSE        Failure

/// Set IP address and port number according to server IP address and port number

//#define SERVER_IP_ADDRESS    "192.168.27.1"       // SERVER LAPTOP
//#define SERVER_IP_ADDRESS    "169.254.189.208"  // SERVER SCARA
#define SERVER_IP_ADDRESS    "192.168.27.50"  // SERVER SCARA

#define PORT_NUMBER          8000
//#define BUFFER_SIZE          21
#define BUFFER_SIZE          128
#define BUFFER_SIZE_JSON     128 

//******************************************************************************
// Global variables

bool stop1 = false;						// stop flag for MainThread
bool offsetFlag = true;					// flag used to offset yaw the first time MainThread is executed
char OPTION;

//******  UART  ******
HANDLE portHandle;
HANDLE portHandle2;
HANDLE portHandle3;
DWORD noOfBytesRead = 0;
DWORD noOfBytesRead2 = 0;
DWORD noOfBytesRead3 = 0;
char receiveBuffer[23] = {0};			// buffer for incoming serial data
char receiveBuffer2[23] = {0};
char receiveBuffer3[28] = {0};
std::vector<double> angles;				// vector of floating point numbers corresponding to the data from the IMU

//******  SOCKETS  *******
WSADATA ws;								///< Structure to contain information about the Windows Socket implementation
SOCKET commSocket;
int retVal_S = 0;
char sendBuffer[BUFFER_SIZE] = {0};
char recvBuffer[BUFFER_SIZE] = {0};
struct sockaddr_in serverinfo;

//***** Human Arm Parameters *****
double theta1 = 0.0;
double theta2 = 0.0;
double theta3 = 0.0;
double phi1 = 0.0;
double phi2 = 0.0;
double phi3 = 0.0;

const double L1 = 0.25;				   // meters  
const double L2 = 0.25;				   // meters
const double L3 = 0.08;				   // meters

double px = 0.0;
double py = 0.0;
double pz = L1 + L2 + L3;
int gripperFlag;
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

double tolerance = 0.1;
//*******************************

//********** Datalog *************
std::vector<double> log_px;
std::vector<double> log_py;
std::vector<double> log_pz;
std::vector<int> log_g;


// Contador para o número de iterações do loop da MainThread()
int i = 1;

//*******************************************************************************************************
 
//**************************************************************************************************
// Main Thread - deals with serial data and kinematics

DWORD WINAPI MainThread(LPVOID pParam)
	{
		 		
		//***** Variables for Timing *****
		DWORD dwOldTime;
		DWORD dwTimeElapsed;
		//********************************
		
		//if (OPTION == '1'){
                for (size_t i = 0; i < log_px.size(); i++)
                {						
					dwOldTime = GetTickCount();    // starts time counter
                   
					px = log_px[i];
					py = log_py[i];
					pz = log_pz[i];
					gripperFlag = log_g[i];
					//std::cout << "px: " << px << ", py: " << py << ", pz: " << pz << std::endl;
				


					// {"Command":"Impedance","X":0.3571,"Y":0.3886,"Z":-0.0351}
					//sprintf(sendBuffer,"{\"Command\":\"Impedance\",\"X\":%.4lf,\"Y\":%.4lf,\"Z\":%.4lf}",px,py,pz);
					sprintf(sendBuffer,"{\"Command\":\"Impedance\",\"X\":%.4lf,\"Y\":%.4lf,\"Z\":%.4lf,\"Gripper\":%d}",px,py,pz,gripperFlag);
					//sprintf(sendBuffer,"{\"Command\":\"Impedance\",\"X\":%.4lf,\"Y\":%.4lf,\"Z\":%.4lf}, %d",px,py,pz,i);
					
					//sprintf(sendBuffer, "%.3lf;%.3lf;%.3lf\n",px,py,pz);
					
					//std::cout << sendBuffer << std::endl;
					    //sprintf(sendBuffer, "%f,%f,%f,%f\n",theta1,phi1,theta2,phi2);
						//sprintf(sendBuffer, "%lf, %lf, %lf %d\n",px,py,pz,i);					
					
					
					retVal_S = send(commSocket, sendBuffer, BUFFER_SIZE, 0);
					 //retVal_S = send(commSocket, buffer.GetString(), BUFFER_SIZE_JSON, 0);
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
			//}
			/*if (OPTION == '2'){
                while( stop1 == false )
                {
					std::cout << "repeating..." << std::endl;
					Sleep(20);
				}
			}
			if (OPTION != '1' && OPTION != '2'){
				std::cout << "waiting..." << std::endl;
				Sleep(100);
			}*/
                return 0;  
	}

// **************************************************************************************************


//******************************************************************************
int wmain(void)
{
    

	DWORD ThreadID;
	HANDLE hMainThread;
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
 

            printf("\n\n\n*****************************************\n");
			printf("Welcome to WITRA: Wearable Interface for Teleoperation of Robot Arms\n");
            printf("*****************************************\n\n");
			printf("REPEATING LAST TELEOPERATION MOVEMENTS");
			//printf("[1] START TELEOPERATION\n");
			//printf("[2] REPEAT RECORDED TELEOPERATION\n\n");
			//printf("OPTION: ");
			//scanf("%d", &OPTION);
			//std::cin.get(OPTION);
			
           
			//OPTION = 2;

			std::ifstream logx("px.txt");
			std::ifstream logy("py.txt");
			std::ifstream logz("pz.txt");
			std::ifstream logg("gripper.txt");

			if(!logx){
				std::cerr << "Couldn't open 'px.txt'\n";
				
			}
			if(!logy){
				std::cerr << "Couldn't open 'py.txt'\n";
				
			}
			if(!logz){
				std::cerr << "Couldn't open 'pz.txt'\n";
				
			}
			if(!logg){
				std::cerr << "Couldn't open 'gripper.txt'\n";
				
			}

			double temp_x;
			double temp_y;
			double temp_z;
			int temp_g;

			while (logx >> temp_x) {
				log_px.push_back(temp_x);
			}
			while (logy >> temp_y) {
				log_py.push_back(temp_y);
			}
			while (logz >> temp_z) {
				log_pz.push_back(temp_z);
			}
			while (logg >> temp_g) {
				log_g.push_back(temp_g);
			}

			

			// Creates the Main Thread 
			hMainThread = CreateThread( NULL,
										   0,
										   MainThread,
										   NULL, // parâmetro passado para a thread
										   0,
										   &ThreadID );
			

        
					getchar();
					stop1 = true;

					//if (OPTION == 1){
						// Save data to datalog



					//}


					CloseHandle(hMainThread);
					getchar();  //usado se retirar o StopThread
					closesocket(commSocket);
					WSACleanup();  
					
					return 0;
    
}
 