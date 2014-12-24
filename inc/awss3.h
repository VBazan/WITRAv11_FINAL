/// @file       awss3.h
/// @copyright  Copyright (c) 2012 Toradex AG
///             [Software License Agreement]
/// $Author: agp $
/// $Rev: 1179 $
/// $Date: 2012-12-06 06:08:53 +0100 (Do, 06 Dez 2012) $
/// @brief      Contains function declarations to be used to access AWS Cloud S3 Services
/// @target     Colibri Modules [PXAxxx, Tegra T20], WinCE 6.0 / 7.0
/// @caveats    

#ifndef _AWSS3_H_
#define _AWSS3_H_

// Header files
#include <windows.h>

#define AWS_S3_ACCESS_ID_MAX_LENGTH         20   ///< 20 Characters
#define AWS_S3_SECRET_KEY_MAX_LENGTH        40   ///< 40 Characters

#define MAX_RESOURCE_PATH_SIZE              1024 ///< Max Resource Path String Length allowed
#define AWS_S3_TIME_STRING_LEN              32   ///< AWS Time String Length
#define AWS_SIGNATURE_STRING_LEN            28   ///< Length of Signature String
#define AWS_LIST_ALL_BUCKET_CHUNK_SIZE      (1024 * 32) 
#define AWS_LIST_BUCKET_CHUNK_SIZE          (1024 * 32) 
#define AWS_DOWNLOAD_FILE_CHUNK_SIZE        (1024 * 100) 
#define AWS_UPLOAD_FILE_CHUNK_SIZE          (1024 * 100) 
#define S3_HANDLE_MEMORY_ALLOCATION         (3 * 4)
#define S3_HEADER_BUFFER_SIZE               120
#define S3_HTTP_RESPONSE_DATA_BUFFER_SMALL  (1 * 1024)

// Structure to hold S3 Account related attributes
typedef struct
{
    char *s3AccessID;      ///< AWS Account Access ID
    char *s3SecretKey;     ///< AWS Account Secret Key    
    char *s3Path;          ///< S3 path
} s3, *HS3;

/// Call back Function
typedef void(*CallBackType)(DWORD, DWORD);

//******************************************************************************
/// Opens Handle for s3Account
/// @param[out]   s3Account     s3Account Handle
/// @param[in]    accessId      Pointer to the aws account Access Id string [20 Characters]
/// @param[in]    secretKey     Pointer to the aws account Secret Key [40 Characters]
/// @param[in]    path          Pointer to the aws s3 path string
/// @retval       TRUE          Success
/// @retval       FALSE         See GetLastError()
BOOL S3Open(HS3 *s3Account, const char *accessId, const char *secretKey, const char *path);

//******************************************************************************
/// Lists all the bucket in S3 account
/// List is stored in a local file
/// @param[in]    s3Account    Handle returned by call to S3Open()
/// @param[out]   listData     Pointer to the Bucket list\n
///                            Data will be like "Bucket1Name:CreationDate"
///                            User need to allocate sufficient memory to hold the data before using this function\n
///                            [NULL] if this parameter is set NULL, it will give only noOfBuckets & rawBytes as output\n
///                            [pointer variable] after knowing number of buckets/number of raw data bytes required to hold bucket list data\n
///                            User can decide how many bytes are required to allocate for bucket list\n
///                            User can allocate rawBytes number of bytes\n
/// @param[in]    filePath     Pointer to the file path, in which all bucket list data to be stored.
/// @param[out]   rawBytes     Pointer to the variable holding number of bytes required for bucket list data
/// @retval       TRUE         Success
/// @retval       FALSE        See GetLastError()  
/// @caveats      User need to check & if file exists, delete the file(filePath) before using the function
BOOL S3ListAllBuckets(HS3 s3Account, char *listData, const char *filePath, DWORD *rawBytes); 

//******************************************************************************
/// Lists all the files inside a bucket\n
/// List is stored in a local file
/// @param[in]    s3Account    Handle returned by call to S3Open()
/// @param[in]    bucketPath   Pointer to the path of the bucket, for example "/awswebdrive1/"
/// @param[in]    tmpFileName  Pointer to the local file path string in which list data will be stored
/// @param[out]   rawBytes     Pointer to the variable which holds size of file in number of bytes
/// @retval       TRUE         Success
/// @retval       FALSE        See GetLastError()  
/// @caveats      User need to check & if file exists, delete the file(filePath) before using the function
BOOL S3ListBucket(HS3 s3Account, const char *bucketPath, const char *tmpFileName, DWORD *rawBytes);

//******************************************************************************
/// Finds Bucket details one by one from list stored in a local file\n
/// File should be initialized by calling S3ListAllBuckets()
/// @param[in]     filePath           Pointer to the file path string
/// @param[in]     fileSize           Size of the file in bytes, this value is received from call to S3ListAllBuckets() in variable rawBytes
/// @param[out]     dataOut            Pointer to the memory holding file information: "filename:size of file",\n
///                                   User need to allocate memory to hold data
/// @param[in]     dataOutSize        Size of the memory allocated for dataOut
/// @param[in, out] pointerToNextFile Pointer holds value to next file details, this parameter will be NULL for 1st call
/// @retval        TRUE               Success
/// @retval        FALSE              See GetLastError()
BOOL FindFirstBucket(const char *filePath, DWORD fileSize, char *dataOut, DWORD dataOutSize, DWORD *pointerToNextFile);

//******************************************************************************
/// Finds file inside the bucket one by one, from the bucket list stored in a local file\n
/// File should be initialized by calling S3ListBucket()
/// @param[in]     filePath           Pointer to the file path string
/// @param[in]     fileSize           Size of the file in bytes, this value is received from call to S3ListBucket() in variable rawBytes
/// @param[out]    dataOut            Pointer to the memory holding file information: "filename:size of file", user need to allocate memory to hold data
/// @param[in]     dataOutSize        Size of the memory allocated for dataOut 
/// @param[in, out] pointerToNextFile Pointer holds value to next file received from previous call  to FindFirstFileInBucket(), this parameter will be NULL for 1st call 
/// @retval        TRUE               Success
/// @retval        FALSE              See GetLastError() 
BOOL FindFirstFileInBucket(const char *filePath, DWORD fileSize, char *dataOut, DWORD dataOutSize, DWORD *pointerToNextFile);

//******************************************************************************
/// Makes new directory in a Bucket
/// @param[in]    s3Account      Handle returned by call to S3Open()
/// @param[in]    directoryPath  Pointer to the s3 directory path string  
/// @retval       TRUE           Success
/// @retval       FALSE          See GetLastError() 
/// @caveats      if existing directory path is given, it will report successful
BOOL S3MakeDirectory(HS3 s3Account, const char *directoryPath);

//******************************************************************************
/// Deletes existing directory from a Bucket
/// @param[in]    s3Account      Handle returned by call to S3Open()
/// @param[in]    directoryPath  Pointer to the s3 directory path string  
/// @retval       TRUE           Success
/// @retval       FALSE          See GetLastError() 
/// @caveats      if non existing directory path is given, it will report success
BOOL S3RemoveDirectory(HS3 s3Account, const char *directoryPath);

//******************************************************************************
/// Deletes existing file from a Bucket
/// @param[in]    s3Account Handle returned by call to S3Open()
/// @param[in]    filePath  Pointer to the s3 file path string  
/// @retval       TRUE      Success
/// @retval       FALSE     See GetLastError()
/// @caveats      if filePath does not exists, it will report success
BOOL S3DeleteFile(HS3 s3Account, const char *filePath);

//******************************************************************************
/// Downloads a file from a bucket to local system
/// @param[in]    s3Account            Handle returned by call to S3Open()
/// @param[in]    sourceFilePath       Pointer to the remote/s3 file path string 
/// @param[in]    destinationFilePath  Pointer to the local file path string 
/// @param[in]    fileSize             Optional  to use, Size of the file to download, user need to find out using call to S3ListAllBuckets()\n
///                                    if doesn't want to use this feature put NULL
/// @param[out]   bytesDownloaded      Pointer to the variable which holds number of bytes downloaded
/// @param[in]    callBack             Pointer to the function which will be called during download in progress\n
///                                    Function will be called, after every 100KB of data is downloaded
/// @retval       TRUE                 Success
/// @retval       FALSE                See GetLastError()
/// @caveats      "+" character is not allowed in sourceFilePath & destinationFilePath
BOOL S3DownloadFile(HS3 s3Account, const char *sourceFilePath, const char *destinationFilePath,
                    DWORD fileSize, DWORD *bytesDownloaded, CallBackType callBack);

//******************************************************************************
/// Uploads a file to a bucket, existing on a local system
/// @param[in]    s3Account            Handle returned by call to S3Open()
/// @param[in]    sourceFilePath       Pointer to the local file path string 
/// @param[in]    sourceFileSize       size of the file in bytes (sourceFilePath)
/// @param[in]    destinationFilePath  Pointer to the remote/S3 file path string 
/// @param[out]   bytesUploaded        Pointer to the variable which will holds, number of bytes uploaded
/// @param[in]    callBack             Pointer to the function which will be called during upload in progress\n
///                                    Function will be called after, every 100KB of data is uploaded
/// @retval       TRUE                 Success
/// @retval       FALSE                See GetLastError()
/// @caveats      "+" character is not allowed in sourceFilePath & destinationFilePath
BOOL S3UploadFile(HS3 s3Account, const char *sourceFilePath, DWORD sourceFileSize, 
                  const char *destinationFilePath, DWORD *bytesUploaded, CallBackType callBack);

//******************************************************************************
/// Deletes already opened s3Account Handle
/// @param[in]    s3Account Handle returned by call to S3Open()
/// @retval       TRUE      Success
/// @retval       FALSE     See GetLastError()  
BOOL S3Close(HS3 *s3Account);
#endif //_AWSS3_H_