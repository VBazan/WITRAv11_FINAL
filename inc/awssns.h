/// @file       awssns.h
/// @copyright  Copyright (c) 2012 Toradex AG
///             [Software License Agreement]
/// $Author: agp $
/// $Rev: 1179 $
/// $Date: 2012-12-06 06:08:53 +0100 (Do, 06 Dez 2012) $
/// @brief      Contains function declarations to be used to access AWS Cloud Sns Services
/// @target     Colibri Modules [PXAxxx, Tegra T20], WinCE 6.0 / 7.0
/// @caveats    Number of Topics are restricted to 100, number of subscriptions per Topic are limited to 100

#ifndef _AWSSNS_H_
#define _AWSSNS_H_

// Header files
#include <windows.h>

#define AWS_SNS_ACCESS_ID_MAX_LENGTH        20  ///<AWS Account ID string Length
#define AWS_SNS_SECRET_KEY_MAX_LENGTH       40  ///<AWS Account Secret Kay String Length
#define AWS_SNS_OWNER_ID_MAX_LENGTH         12  ///<AWS Account Owner ID String Length

#define SNS_TOPIC_NAME_MAX_LENGTH           256 ///<Maximum number of characters allowed in Topic Name
#define SNS_DISPLAY_NAME_MAX_LENGTH         100 ///<Maximum number of characters allowed in Topic's Display Name
#define SNS_MAX_TIME_STRING_LENGTH          24  ///<AWS Sns Formatted Time String Length
#define AWS_SNS_SIGNATURE_HASH_LEN          32  ///<HMAC-SHA2 HASH Length
#define AWS_SNS_SIGNATURE_STRING_LEN        44  ///<HMAC-SHA2 Signature String Length

#define AWS_SNS_MESSAGE_SIZE_MAX            (32 * 1024) ///<Maximum allowed message length 32 KB (Applicable to SnsPublish)
#define AWS_SNS_SUBJECT_MAX                 (100)       ///<Maximum number of characters allowed in subject (Applicable to SnsPublish)
#define SNS_LIST_TOPIC_RECEIVE_CHUNK        (32 * 1024) ///<Buffer Size used when receiving list of Topics
#define SNS_LIST_SUBSCRIPTION_RECEIVE_CHUNK (32 * 1024) ///<Buffer Size used when receiving list of Subscriptions

#define PROTOCOL_TYPE_HTTP                  1 ///<http Protocol
#define PROTOCOL_TYPE_HTTPS                 2 ///<https Protocol
#define PROTOCOL_TYPE_EMAIL                 3 ///<email Protocol
#define PROTOCOL_TYPE_EMAIL_JSON            4 ///<email-json Protocol
#define PROTOCOL_TYPE_SMS                   5 ///<sms Protocol
#define PROTOCOL_TYPE_SQS                   6 ///<sqs Protocol

#define ATTRIBUTE_TYPE_POLICY               1 ///<Attribute Type Selection Policy
#define ATTRIBUTE_TYPE_DISPLAYNAME          2 ///<Attribute Type Selection Display Name
#define ATTRIBUTE_TYPE_DELIVERYPOLICY       3 ///<Attribute Type Selection Delivery Policy

#define SNS_HANDLE_MEMORY_ALLOCATION_SIZE   (7 * 4)

#define SNS_HTTP_RESPONSE_DATA_BUFFER_SMALL (1 * 1024)
#define SNS_HTTP_RESPONSE_DATA_BUFFER_LARGE (3 * 1024)
#define SNS_HTTP_ARGUMENT_BUFFER_SMALL      (2 * 1024)
#define SNS_STRING_TO_SIGN_BUFFER_SMALL     (2 * 1024)

#define URL_ENCODED_TOPIC_NAME_BUFFER       1024
#define URL_ENCODED_TOPIC_ARN_BUFFER        1024
#define URL_ENCODED_SUB_ARN_BUFFER          1024
#define URL_ENCODED_ENDPOINT_BUFFER         1024
#define URL_ENCODED_SUBJECT_BUFFER          512
#define URL_ENCODED_ATTRIBUTE_VALUE_BUFFER  512

#define PROTOCOL_STRING_LENGTH              10
#define ATTRIBUTE_VALUE_BUFFER_SIZE         256

// Structure to hold Sns Account & Topic related attributes
typedef struct
{
    char *snsAccessID;      ///<Pointer to hold address of AWS Account AccessId string
    char *snsSecretKey;     ///<Pointer to hold address of AWS Account SecretKey string
    char *snsPath;          ///<Pointer to hold address of AWS Account Sns region Path string
    char *snsTopicName;     ///<Pointer to hold address of AWS Account AccessID string
    char *snsTopicAmazonResourceName; ///<Pointer to hold address of AWS Account AccessID string
    char *snsDisplayName;   ///<Pointer to hold address of Topic Display Name
    char *snsOwnerId;       ///<Pointer to hold address of AWS Account OwnerId string
} snsTopic, *HSNS;

//******************************************************************************
/// Opens handle for Sns topic, the returned snsTopicHandle is required as an input parameter to every Sns function call
/// @param[in]    accessID      AWS Access ID [20 Characters]
/// @param[in]    secretKey     AWS Secret Key [40 Characters]
/// @param[in]    ownerId       AWS Owner ID [12 Characters]
/// @param[in]    path          AWS Sns path
/// @param[in]    topicName     Topic names must be constructed using only uppercase and lowercase ASCII letters,\n
///                             numbers, underscore, hyphens and must be between 1 and 256 characters long
/// @param[in]    displayName   [OPTIONAL] if not NULL, Display names must be constructed using only uppercase/lowercase ASCII letters,\n
///                             numbers, underscore, hyphens and must be between 1 and 100 characters long
/// @param[out]   snsTopicHandle Structure to hold various sns related parameters.
/// @retval       TRUE          Success
/// @retval       FALSE         See GetLastError()
BOOL SnsOpenTopic(char *accessID, char *secretKey, char *ownerId, char *path, char *topicName, char *displayName, HSNS *snsTopicHandle);

//******************************************************************************
/// Deletes already created Sns Topic Handle
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
BOOL SnsCloseTopic(HSNS *snsTopicHandle);

//******************************************************************************
/// Creates new Sns Topic
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
BOOL SnsCreateTopic(HSNS snsTopicHandle);

//******************************************************************************
/// Publishes/Sends a message to all the subscribed endpoints of a topic. When a messageId is returned,\n
/// the message has been saved and Amazon Sns will attempt to deliver it to the topic subscribers shortly\n
/// The format of the outgoing message to each subscribed endpoint depends on the notification protocol selected
/// @param[in]     snsTopicHandle   Handle returned by call to SnsTopicOpen()
/// @param[in]     subject          Optional parameter to be used as the "Subject" line, when the message is delivered to e-mail endpoints, max 100 ASCII characters
/// @param[in]     message          The message you want to send to the topic. Maximum length is 32 x 1024 characters
/// @retval        TRUE             Success
/// @retval        FALSE            See GetLastError()
BOOL SnsPublish(HSNS snsTopicHandle, const char *subject, const char *message);

//******************************************************************************
/// Deletes already existing Sns Topic
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
BOOL SnsDeleteTopic(HSNS snsTopicHandle);

//******************************************************************************
/// Subscribes an endpoint to a topic
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @param[in]    endPoint          URL "http://", URL "https://" , Email ID, Phone number for SMS, SQS: ARN of an Amazon SQS queue, etc
/// @param[in]    protocolType      Valid Values can be from 1 to 6, \n[1] - "http", \n[2] - "https", \n[3] - "email", \n[4] - "email-json", \n[5] - "sms", \n[6] - "sqs"
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
BOOL SnsSubscribe(HSNS snsTopicHandle, char *endPoint, BYTE protocolType);

//******************************************************************************
/// Lists topics in an Sns Account, for a particular region (max 100)\n
/// Topics are stored into a local file
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @param[in]    filePath          File in which list of topics will be saved, user need to delete already existing file,\n
///                                 else list data will be appended in the same file
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
/// @caveats      Number of Topics are restricted to 100, number of subscriptions per Topic are limited to 100\n
BOOL SnsListTopics(HSNS snsTopicHandle, char *filePath);

//******************************************************************************
/// Unsubscribes an endpoint from its subscription
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @param[in]    subArn            Subscription Arn String
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
BOOL SnsUnsubscribe(HSNS snsTopicHandle, char *subArn);

//******************************************************************************
/// Lists Subscriptions by topic in an Sns Account, for a particular region (max 100)\n
/// Subscriptions by Topic are stored into a local file
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @param[in]    filePath          File in which list of Subscriptions by topics will be saved, user need to delete already existing file,\n
///                                 else list data will be appended in the same file
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
/// @caveats      Number of Topics are restricted to 100, number of subscriptions per Topic are limited to 100\n
BOOL SnsListSubscriptions(HSNS snsTopicHandle, char *filePath);

//******************************************************************************
/// Finds Topic Details from list of Topics stored in a local file\n
/// The file must be initialized by calling SnsListTopics()
/// @param[in]       filePath           File path in which list of subscription exists
/// @param[out]      dataOut            Pointer to the output string containing Topic details, user need to allocate memory
/// @param[in]       dataOutSize        Number of bytes allocated for dataOut
/// @param[in, out]  pointerToNextTopic Pointer to next Topic details from list data, to find 1st topic details, put NULL.
/// @param[in]       fileSize           Size of the file in bytes
/// @retval          TRUE               Success
/// @retval          FALSE              See GetLastError()
BOOL SnsFindTopic(const char *filePath, char *dataOut, DWORD dataOutSize, DWORD *pointerToNextTopic, DWORD fileSize);

//******************************************************************************
/// Finds Subscription details by Topic from list of subscriptionsByTopic stored in a local file\n
/// The file must be initialized by calling SnsListSubscriptionsByTopic()
/// @param[in]       filePath       File path in which list of subscription exists
/// @param[out]      dataOut        Pointer to the output string containing subscription details, user need to allocate memory
/// @param[in]       dataOutSize    Number of bytes allocated for dataOut
/// @param[in, out]  pointerToNextSubscriptionByTopic   Pointer to next Subscription details from list data, to find 1st Subscription by topic Detail, put NULL
/// @param[in]       fileSize       Size of the file in bytes
/// @retval          TRUE           Success
/// @retval          FALSE          See GetLastError()
BOOL SnsFindSubscriptions(const char *filePath, char *dataOut, DWORD dataOutSize, DWORD *pointerToNextSubscriptionByTopic, DWORD fileSize);

//******************************************************************************
/// Gets Subscription Attributes
/// @param[in]     snsTopicHandle   Handle returned by call to SnsTopicOpen()
/// @param[in]     subArn           Subscription Arn String
/// @param[out]    dataOut          Pointer to the output string containing subscription attributes details, user need to allocate memory
/// @param[in]     dataOutSize      Number of bytes allocated for dataOut
/// @retval        TRUE             Success
/// @retval        FALSE            See GetLastError()
BOOL SnsGetSubscriptionAttribtues(HSNS snsTopicHandle, char *subArn, char *dataOut, DWORD dataOutSize);

//******************************************************************************
/// Gets Topic Attributes of Sns Topic
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @param[out]   dataOut           Pointer to the output string containing TopicAttribute details, user need to allocate memory
/// @param[in]    dataOutSize       Number of bytes allocated for dataOut
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
BOOL SnsGetTopicAttributes(HSNS snsTopicHandle, char *dataOut, DWORD dataOutSize);

//******************************************************************************
/// Sets Topic Attributes
/// @param[in]    snsTopicHandle    Handle returned by call to SnsTopicOpen()
/// @param[in]    attributeType     [1] - Policy, \n[2] - DisplayName, \n[3] - DeliveryPolicy
/// @param[in]    attributeValue    String containing respective attribute value
/// @retval       TRUE              Success
/// @retval       FALSE             See GetLastError()
/// @caveats      only attributes = 2 (DisplayName) is working
BOOL SnsSetTopicAttributes(HSNS snsTopicHandle, BYTE attributeType, char *attributeValue);

#endif // _AWSSNS_H_