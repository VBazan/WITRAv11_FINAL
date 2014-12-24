/// @file       awssqs.h
/// @copyright  Copyright (c) 2012 Toradex AG
///             [Software License Agreement]
/// $Author: kia $
/// $Rev: 1327 $
/// $Date: 2013-02-01 11:35:26 +0100 (Fr, 01 Feb 2013) $
/// @brief      Contains function declarations to be used to access AWS Cloud SQS Services
/// @target     Colibri Modules [PXAxxx, Tegra T20], WinCE 6.0 / 7.0
/// @caveats

#ifndef _AWSSQS_H_
#define _AWSSQS_H_

// Header files
#include <windows.h>

#define AWS_SQS_ACCESS_ID_MAX_LENGTH                20 ///<AWS Account ID string Length
#define AWS_SQS_SECRET_KEY_MAX_LENGTH               40 ///<AWS Account Secret Kay String Length
#define AWS_SQS_OWNER_ID_MAX_LENGTH                 12 ///<AWS Account Owner ID String Length
#define AWS_SQS_SIGNATURE_HASH_LENGTH               32
#define AWS_SQS_SIGNATURE_STRING_LENGTH             44

#define SQS_QUEUE_NAME_MAX_LENGTH                   80
#define SQS_MAX_TIME_STRING_LENGTH                  24
#define SQS_MESSAGE_ID_LENGTH                       100
#define SQS_MESSAGE_DELAY_SECONDS_MAX               900
#define SQS_MESSAGE_BODY_MAX_LENGTH                 (64 * 1024)
#define SQS_RECEIPT_HANDLE_MAX_LENGTH               1024
#define SQS_ATTRIBUTE_NAME_BUFFER_SIZE              24

#define SQS_HTTP_RESPONSE_DATA_BUFFER_SMALL         (2 * 1024)
#define SQS_HTTP_RESPONSE_DATA_BUFFER_LARGE         (4 * 1024)
#define SQS_HTTP_ARGUMENT_BUFFER_SMALL              (2 * 1024)
#define SQS_HTTP_ARGUMENT_BUFFER_LARGE              (4 * 1024)
#define SQS_STRING_TO_SIGN_BUFFER_SMALL             (2 * 1024)
#define SQS_STRING_TO_SIGN_BUFFER_LARGE             (4 * 1024)

#define URL_ENCODED_RECEIPT_HANDLE_BUFFER           (4 * 1024)

#define SQS_HANDLE_MEMORY_ALLOCATION                (4 * 5)

#define QUEUE_ATTRIBUTE_VISIBILITY_TIMEOUT          1
#define QUEUE_ATTRIBUTE_MAXIMUM_MESSAGE_SIZE        2
#define QUEUE_ATTRIBUTE_MESSAGE_RETENTION_PERIOD    3
#define QUEUE_ATTRIBUTE_DELAY_SECONDS               4

// Structure to hold Sqs Account & queue related attributes
typedef struct
{
    char *sqsAccessID;      ///< AWS Account Access ID
    char *sqsSecretKey;     ///< AWS Account Secret Key    
    char *sqsPath;          ///< SQS Queue path
    char *sqsAccountOwnerID;///< AWS Account owner Id
    char *sqsQueueName;     ///< SQS Queue Name
} sqsQueue, *HSQS;

//******************************************************************************
/// Opens Handle for SQS Queue, the returned sqsQueue Handle is required as an input parameter to every Sqs function call
/// @param[in]      accessID          AWS Access ID [20 Characters]
/// @param[in]      secretKey         AWS Secret Key [40 Characters]
/// @param[in]      path              AWS SQS path
/// @param[in]      accountOwnerID    12 digit unique ID number like, AWS Access ID
/// @param[in]      queueName         Queue names must be constructed using only uppercase/lowercase ASCII letters,\n
///                                   numbers, underscore, hyphen, and must be between 1 and 80 characters long.
/// @param[out]     sqsQueue          Handle to hold various sqs related parameters.
/// @retval         TRUE              Success
/// @retval         FALSE             See GetLastError()  
BOOL SqsOpenQueue(char *accessID, char *secretKey, char *path, char *accountOwnerID, char *queueName, HSQS *sqsQueue);

//******************************************************************************
/// Deletes already opened SQS Queue Handle
/// @param[in]      sqsQueue  Handle returned by call to SqsOpenQueue()
/// @retval         TRUE      Success
/// @retval         FALSE     See GetLastError()  
BOOL SqsCloseQueue(HSQS *sqsQueue);

//******************************************************************************
/// Sends a message to the Queue
/// @param[in]      sqsQueue        Handle returned by call to SqsOpenQueue()
/// @param[in]      message         The message you want to send to the queue. Maximum length is 64KB
/// @param[in]      delaySeconds    Number of seconds to delay the message\n
///                                 Messages with a positive delay seconds becomes available for processing after the delay time has elapsed\n
/// @param[out]     messageID       Successful call to this function will put message ID string on the memory pointed by this pointer, user need to allocate memory
/// @param[in]      messageIDSize   Size of the memory allocated for messageID
/// @retval         TRUE            Success
/// @retval         FALSE           See GetLastError()  
BOOL SqsSendMessage(HSQS sqsQueue, char *message, DWORD delaySeconds, char *messageID, DWORD messageIDSize);

//******************************************************************************
/// Receives message from an existing Queue
/// @param[in]      sqsQueue             Handle returned by call to SqsOpenQueue()
/// @param[in]      attributeName        Valid String values: All | SenderId | SentTimestamp | ApproximateReceiveCount | ApproximateFirstReceiveTimestamp\n
/// @param[in]      maxNumberOfMessages  Maximum number of messages to return, 1 to 10(maximum)
/// @param[in]      visibilityTimeout    The duration (in seconds) that the received messages are hidden\n
///                                      from subsequent retrieve requests after being retrieved by a SqsReceiveMessage() request.
/// @param[out]     message              Pointer to the received Message as string (64KB maximum)
/// @param[out]     messageID            Pointer to the received Message's ID as string, maximum length = 100
/// @param[out]     receiptHandle        Pointer to the receiptHandle as string, it is required by SnsDeleteMessage() call. Maximum length = 1024
/// @retval         TRUE                 Success
/// @retval         FALSE                See GetLastError()  
/// @caveats        AttributeName, visibilityTimeout, maxNumberOfMessages - working in default mode only for now (kept it for future use)\n
///                 Put NULL for these input parameters
BOOL SqsReceiveMessage(HSQS sqsQueue, char *attributeName, BYTE maxNumberOfMessages, DWORD visibilityTimeout, char *message, char *messageID, char *receiptHandle);

//******************************************************************************
/// Deletes the Queue
/// @param[in]      sqsQueue    Handle returned by call to SqsOpenQueue()
/// @retval         TRUE        Success
/// @retval         FALSE       See GetLastError()  
BOOL SqsDeleteQueue(HSQS sqsQueue);

//******************************************************************************
/// Creates a Queue
/// @param[in]      sqsQueue    Handle returned by call to SqsOpenQueue()
/// @retval         TRUE        Success
/// @retval         FALSE       See GetLastError() 
BOOL SqsCreateQueue(HSQS sqsQueue);

//******************************************************************************
/// Gets Queue Url
/// @param[in]      sqsQueue    Handle returned by call to SqsOpenQueue()
/// @param[out]     dataOut     Pointer to the allocated memory to hold Queue string returned by the function, user need to allocate memory
/// @param[in]      dataOutSize Size of the allocated memory for dataOut
/// @retval         TRUE        Success
/// @retval         FALSE       See GetLastError() 
BOOL SqsGetQueueUrl(HSQS sqsQueue, char *dataOut, DWORD dataOutSize);

//******************************************************************************
/// Lists all Queues in a local file starting with prefix mentioned\n
/// Queues are stores in a local file
/// @param[in]      sqsQueue        Handle returned by call to SqsOpenQueue()
/// @param[in]      queueNamePrefix Prefix example "test" to get a list of all queues starting with test or\n
///                                 "t" to get list of all queue starting with letter t
/// @param[in]      filePath        Local file path in which queue list data will be saved
/// @retval         TRUE            Success
/// @retval         FALSE           See GetLastError() 
BOOL SqsListQueues(HSQS sqsQueue, char *queueNamePrefix, char *filePath);

//******************************************************************************
/// Finds one by one Queue details stored in a local file as a result of function call to SnsListQueues()\n 
/// File must be initialized by calling SqsListQueues()
/// @param[in]      filePath            Local file path in which queue list data is saved
/// @param[out]     dataOut             Pointer to the allocated memory to store returned queue detail, user need to allocate memory
/// @param[in]      dataOutSize         Size of the allocated memory pointed by dataOut
/// @param[in, out] pointerToNextTopic  Points to next location of detail, to find 1st in the list put NULL
/// @param[in]      fileSize            Size of the file in bytes, pointed by filePath
/// @retval         TRUE                Success
/// @retval         FALSE               See GetLastError() 
BOOL SqsFindQueue(const char *filePath, char *dataOut, DWORD dataOutSize, DWORD *pointerToNextTopic, DWORD fileSize);

//******************************************************************************
/// Gets Queue Attributes
/// @param[in]      sqsQueue        Handle returned by call to SqsOpenQueue()
/// @param[out]     dataOut         Pointer to the allocated memory to hold Queue attributes data returned by function, user need to allocate memory
/// @param[in]      dataOutSize     Size of the allocated memory
/// @retval         TRUE            Success
/// @retval         FALSE           See GetLastError() 
BOOL SqsGetQueueAttributes(HSQS sqsQueue, char *dataOut, DWORD dataOutSize);

//******************************************************************************
/// Sets Queue Attribute
/// @param[in]      sqsQueue        Handle returned by call to SqsOpenQueue()
/// @param[in]      attributeName   Valid values can be from 1 to 4, user should use macros QUEUE_ATTRIBUTE_XXXXXXXXX
/// @param[in]      attributeValue  Value associated with attributeName
/// @retval         TRUE            Success
/// @retval         FALSE           See GetLastError() 
BOOL SqsSetQueueAttributes(HSQS sqsQueue, DWORD attributeName, int attributeValue);

//******************************************************************************
/// Deletes a Message from a Queue
/// @param[in]      sqsQueue        Handle returned by call to SqsOpenQueue()
/// @param[in]      receiptHandle   Pointer to the string returned by call to SqsReceiveMessage()
/// @retval         TRUE            Success
/// @retval         FALSE           See GetLastError() 
BOOL SqsDeleteMessage(HSQS sqsQueue, char *receiptHandle);

//******************************************************************************
/// Changes Message's Visibility Timeout attribute
/// @param[in]      sqsQueue                Handle returned by call to SqsOpenQueue()
/// @param[in]      receiptHandle           Pointer to the string returned by call to SqsReceiveMessage()
/// @param[in]      visibilityTimeoutValue  New value for timeout, Range [0 - 900] Seconds
/// @retval         TRUE                    Success
/// @retval         FALSE                   See GetLastError()
BOOL SqsChangeMessageVisibility(HSQS sqsQueue, char *receiptHandle, DWORD visibilityTimeoutValue);
#endif // _AWSSQS_H_