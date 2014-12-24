///
/// @file        UpdateLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: bis $
/// $Rev: 2145 $ 
/// $Date: 2014-04-07 17:28:03 +0200 (Mo, 07 Apr 2014) $
/// @brief       This library allows you to backup and update (restore) one or 
///              multiple flash regions. \n 
///              At the moment the following regions are supported: \n 
///              Bootloader, Config Block, OS Image, Registry, FileSystem.
/// 
/// @target      PXAxxx,T20
/// @caveats     When updating multiple flash regions these regions have to be
///              contiguous. \n It's not allowed to backup e.g. Bootloader and
///              OS Image without backing-up the Config Block! \n
///              Only one store with the profile name "Internal Flash" is
///              allowed when performing Flash filesystem backup/update.
/// 


#ifndef _UPDATELIB_H_
#define _UPDATELIB_H_

#ifdef __cplusplus
extern "C" {
#endif

///Version Info
#define UDL_VER_MAJ     1    ///< Major Library Version
#define UDL_VER_MIN     7    ///< Library Subversion
#define UDL_VER_BUILD   0    ///< Library Build


/// UpdateLib type definitions

typedef enum
{
    UDL_NO_REGION      = 0,
    // Flash regions
    UDL_BOOTLOADER     = 1 << 0,
    UDL_CONFIG_BLOCK   = 1 << 1,
    UDL_OS_IMAGE       = 1 << 2,
    UDL_REGISTRY       = 1 << 3,
    UDL_FILE_SYSTEM    = 1 << 4,
    UDL_2ND_BOOTLOADER = 1 << 5,
    // Insert new FLASH regions here and increment UDL_MAX_REGIONS below.

    // Non-Flash regions
    UDL_CPLD           = 1 << 10,
    //Insert new non-Flash regions here and increment UDL_MAX_REGIONS below.

    // Special regions
    UDL_ANY_BIN_REGION = 1 << 31,  ///< General, unknown region.
    UDL_ALL_REGIONS    = 0xFFFFFFFF,
} 
UDL_REGION_ID;

#define UDL_MAX_REGIONS 8      ///< Region count + 1, adjust this value when inserting new regions.

// Defines for flags field in UDL_REGIONINFO struct.
#define UDL_F_INVALID_REGION  (1 << 0)  ///< This region is invalid (i.e. not found).
#define UDL_F_FAILSAFE        (1 << 1)  ///< Indicates that the fail safe feature is enabled.
#define UDL_F_USE_ECC         (1 << 2)  ///< Indicates to try to use ECC to read this region.

typedef struct {
    UDL_REGION_ID  id;
    DWORD          start;
    DWORD          length;
    DWORD          usedBytes;
    DWORD          flags;
} UDL_REGIONINFO;


typedef enum
{
    UDL_SUCCESS = 0,                    ///< No Error, Function returned sucessfully.
    UDL_ERROR,                          ///< General Error.
    UDL_USER_ABORT,                     ///< The user aborted this function (e.g. via callback function).
    UDL_NOT_INITIALIZED,                ///< The update lib (UDL) is not initialized.
    UDL_HW_NOT_SUPPORTED,               ///< The current HW (module) is not supported.
    UDL_INVALID_FILE_PATH,              ///< The file path is corrupt or file is nonexistent.
    UDL_INVALID_BIN_FILE,               ///< The bin file is corrupt or format not supported.
    UDL_INVALID_FILE_VERSION,           ///< The bin file version is not supported by this lib.
    UDL_FILE_WRITE_ERROR,               ///< Could not write to the file (e.g. disk is full).
    UDL_INSUFFICIENT_MEMORY,            ///< Allocating of memory failed or not enough memory.
    UDL_RANGE_VERIFICATION_FAILED,      ///< The given range overwrites not allowed regions.
    UDL_REGION_NOT_FOUND,               ///< The requested region was not found in flash.
    UDL_REGION_INVALID,                 ///< Cannot backup invalid regions (e.g. possible when FlashFileSystem driver not loaded).
    UDL_FLASH_INIT_ERROR,               ///< Error during initialization of the flash device.
    UDL_FLASH_READ_ERROR,               ///< Error during flash read.
    UDL_FLASH_WRITE_ERROR,              ///< Error during flash write.
    UDL_FLASH_ERASE_ERROR,              ///< Error during flash erase.
    UDL_CRC_ERROR,                      ///< CRC error occured.
    UDL_WRITE_VERIFICATION_FAILED,      ///< The flash verification after writing failed.
    UDL_FILE_SYSTEM_ERROR,              ///< Error during backup or update of FileSystem region.
    UDL_FILE_SYSTEM_DRIVER_LOAD_ERROR,  ///< Error during loading or unloading the file system driver.
    UDL_FILE_SYSTEM_FORMATING_ERROR,    ///< Error during formating of store or partitions.
    UDL_CPLD_NOT_FOUND,                 ///< No CPLD found or CPLD version not supported.
    UDL_CPLD_READ_WRITE_ERROR,          ///< Error during CPLD read or write.
    UDL_DISMOUNT_ERROR                  ///< Dismounting flashdisk error.
} UDL_RET_TYPE;

typedef BOOL (*PUDL_CALBACK_PROC_P1)(void *pParam1);
typedef BOOL (*PUDL_CALBACK_PROC_2)(DWORD param1, DWORD param2);
typedef BOOL (*PUDL_CALBACK_PROC_3)(DWORD param1, DWORD param2, DWORD param3);

//******************************************************************************
/// Returns the library Version.
/// @param[out]       pVerMaj          Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]       pVerMin          Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]       pBuild           Returns the build number.         Set this parameter to NULL if not required.
void UDLGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

//******************************************************************************
/// Initializes the UpdateLib. Updates the internal UDL_REGIONINFO structure for all regions.\n
/// See UDLUpdateRegInfo for more details.
/// @retval            UDL_SUCCESS
/// @retval            UDL_HW_NOT_SUPPORTED
/// @retval            UDL_FLASH_INIT_ERROR
///
/// Remarks:          UDLInit has to be called once before any of the following functions:\n
///                   - UDLSetBinFile\n
///                   - UDLCheckRange\n
///                   - UDLGetRegionInfo\n
///                   - UDLBackupToFile\n
///                   - UDLDoUpdate\n
UDL_RET_TYPE UDLInit(void);

///******************************************************************************
/// DeInitialize the UpdateLib.
/// @retval            UDL_SUCCESS
///
/// Remarks:          The following functions don't work correctly after calling UDLDeInit:\n
///                   - UDLSetBinFile\n
///                   - UDLCheckRange\n
///                   - UDLGetRegionInfo\n
///                   - UDLBackupToFile\n
///                   - UDLDoUpdate.\n
UDL_RET_TYPE UDLDeInit(void);

///******************************************************************************
/// Updates the region info structure that is stored in the UpdateLib. This function is called\n
/// in the UDLInit function to initialize the region info structure. Call this function when\n
/// the region layout in the flash has changed (e.g. Registry start address has changed or\n
/// FileSystem length and/or start address has changed).
/// @param[in]        regID       Defines the region to be updated (useing UDL_ALL_REGIONS is recommended)
/// @retval           UDL_SUCCESS
///
/// Remarks:          Call this function to be sure that the stored region information in the lib gets
///                   updated with the actual region settings in flash.
///                   At the moment there is no check for overlapping of regions, or similar.
UDL_RET_TYPE UDLUpdateRegInfo(UDL_REGION_ID regID);

//******************************************************************************
/// Sets the bin file for a subsequent UDLDoUpdate command and returns some information\n
/// about the content of the bin file.
/// @param[in]        pBinPath            Pointer to a TCHAR string containing the file path of the .bin file\n
///                                       for update process.
/// @param[in]        ReadFileProgress    Function pointer to a callback function which is called during\n
///                                       parsing/validating of the bin file. Set this parameter to NULL if not required.\n
///                                       BOOL ReadFileProgress(DWORD done, DWORD totsize)\n
///                                       - done:    Bytes already read from file\n
///                                       - totsize: Total number of bytes of the file\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue parsing the file.\n
///                                          - FALSE: Stop parsing the file and exit UDLSetBinFile (UDL_USER_ABORT)
/// @param[in]        validate            TRUE:   - Validate the bin file (slower)\n
///                                       FALSE:  - Don't validate the file (faster)
/// @param[out]       pRegInfo            Returns the region info of the complete .bin file (start, length, usedBytes, id)\n
///                                       The pRegInfo.id value can be set to any combination of the following:\n
///                                       - UDL_NO_REGION\n
///                                       - UDL_ANY_BIN_REGION (we don't differentiate all regions, we just return a general id)\n
///                                       - UDL_FILE_SYSTEM\n
///                                       - UDL_CPLD\n
///                                       - UDL_FAILSAFE (Caution! This bin file would overwrite the first bootloader,\n
///                                                       therefore the whole image gets shifted to bl2start\n
///                                                       defined in configblock. See UpdateLibDemo app as reference.)\n
///                                       Set pRegInfo to NULL if not required.
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_INVALID_FILE_PATH
/// @retval            UDL_INVALID_BIN_FILE
/// @retval            UDL_INVALID_FILE_VERSION
/// @retval            UDL_CRC_ERROR
/// @retval            UDL_USER_ABORT
///
/// Remarks:          Call UDLCheckRange or UDLDoUpdate after this call.
///                   The return value can be UDL_SUCCESS although the pRegInfo->id can be UDL_NO_REGION. This is e.g. the
///                   case when the bin file is valid but the current running HW doesn't support a certain bin file
///                   section (E.g. a CPLD update file on a Colibri PXA270).
UDL_RET_TYPE UDLSetBinFile(TCHAR *pBinPath, UDL_REGIONINFO *pRegInfo, PUDL_CALBACK_PROC_2 ReadFileProgress, BOOL validate);

//******************************************************************************
/// Release the bin file we set with UDLSetBinFile if not used anymore.
/// Return Value Type: UDL_RET_TYPE
/// @retval            UDL_SUCCESS      File released
/// @retval            UDL_ERROR        Could not release file
///
/// Remarks:          Call this function after UDLSetBinFile when bin file is not needed anymore (e.g. after UDLDoUpdate).
UDL_RET_TYPE UDLReleaseBinFile(void);

//******************************************************************************
/// Checks the given range and reports an error if a not allowed region gets touched.
/// @param[in]        start:            Start address of the range in flash
/// @param[in]        length:           Length of the range in flash
/// @param[in,out]    pAllowedRegions   In:  Pointer to a UDL_REGION_ID structure that contains the allowed regions\n
///                                          to be touched by the given range\n
///                                     Out: Returns the regions touched by the given range
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_RANGE_VERIFICATION_FAILED
UDL_RET_TYPE UDLCheckRange(UDL_REGION_ID *pAllowedRegions, DWORD start, DWORD length);

//******************************************************************************
/// Returns the information of one or multiple regions (start, length, usedBytes).              
/// @param[in,out]    pRegInfoTotal     In:  pRegInfoTotal->id defines the regions to retrieve the information\n
///                                          The other pRegInfoTotal fields are unused.\n
///                                     Out: pRegInfoTotal contains the information for all given region IDs.
/// @param[out]       pRegInfoArray     The array contains a list of UDL_REGIONINFO structures for all given\n
///                                     region IDs. The list ends with an empty UDL_REGIONINFO\n
///                                     structure (id = UDL_NO_REGION)\n
///                                     This parameter could be set to NULL if not required.
///
/// Return Value Type: UDL_RET_TYPE
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_REGION_NOT_FOUND
UDL_RET_TYPE UDLGetRegionInfo(UDL_REGIONINFO *pRegInfoTotal, UDL_REGIONINFO *pRegInfoArray);

//******************************************************************************
/// Creates a backup of the given regions and saves it into a bin file.
/// @param[in]        pBinPath          Pointer to a TCHAR string containing the file path of the .bin file\n
///                                     for backup process.
/// @param[in]        backupRegions     Defines the regions to backup from flash. UDL_ALL_REGIONS creates a backup\n
///                                     of all valid regions of a device.
/// @param[in]        ImageInfo         Function pointer to a callback function which is called at the beginning\n
///                                     of the backup process returning total image start and size.\n
///                                     Set this parameter to NULL if not required.\n
///                                     BOOL ImageInfo(UDL_REGIONINFO *imgInfo)\n
///                                     - pImgInfo: Pointer to UDL_RETIONINFO structure which includes the region\n
///                                                 IDs, the total start and total length of the backup image.\n
///                                     - Return Value:\n
///                                        - TRUE:  Continue backup process\n
///                                        - FALSE: Stop backup and exit UDLBackUpToFile (UDL_USER_ABORT)
/// @param[in]        SectionInfo       Function pointer to a callback function which is called before backing-up \n
///                                     a new section returning information about the section.\n
///                                     Set this parameter to NULL if not required.\n
///                                     BOOL SectionInfo(DWORD SectionStart, DWORD SectionLen, DWORD SectionCRC)\n
///                                     - SectionStart: Start of the current section\n
///                                     - SectionLen:   Length of the current section\n
///                                     - SectionCRC:   This value is always set to 0.\n
///                                     - Return Value:\n
///                                        - TRUE:  Continue backup process\n
///                                        - FALSE: Stop backup and exit UDLBackUpToFile (UDL_USER_ABORT)
/// @param[in]        WriteProgress     Function pointer to a callback function which is called during\n
///                                     writing the bin file. Set this parameter to NULL if not required.\n
///                                     BOOL WriteProgress(DWORD writtenSectionLen, DWORD writtenTotalLen)\n
///                                     - writtenSectionLen: Number of bytes of the current section which are already\n
///                                                          written to the file.\n
///                                     - writtenTotalLen:   Total number of bytes which are already\n
///                                                          written to the file.\n
///                                     - Return Value:\n
///                                        - TRUE:  Continue backup process\n
///                                        - FALSE: Stop backup and exit UDLBackUpToFile (UDL_USER_ABORT)
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_INVALID_FILE_PATH
/// @retval            UDL_REGION_NOT_FOUND
/// @retval            UDL_REGION_INVALID
/// @retval            UDL_FILE_WRITE_ERROR
/// @retval            UDL_MEM_ALLOC_FAILED
/// @retval            UDL_USER_ABORT
/// @retval            UDL_FLASH_READ_ERROR
/// @retval            UDL_FILE_SYSTEM_ERROR
UDL_RET_TYPE UDLBackupToFile(TCHAR *pBinPath, UDL_REGION_ID backupRegions, PUDL_CALBACK_PROC_P1 ImageInfo, PUDL_CALBACK_PROC_3 SectionInfo, PUDL_CALBACK_PROC_2 WriteProgress);

//******************************************************************************
/// Writes the content of the bin file set by UDLSetBinFile to flash.
/// @param[in]        EraseProgress       Function pointer to a callback function which is called during\n
///                                       erasing of the flash or formatting stores (no feedback during formatting). \n
///                                       Formatting mode: Driver unload/load indication: totsize = 0, done = 1\n
///                                       Formatting mode: Start format inication:        totsize = 0, done = 2\n
///                                       Formatting mode: End format indication:         totsize = 0, done = 3\n
///                                       Set this parameter to NULL if not required.\n
///                                       BOOL EraseProgress(DWORD done, DWORD totsize)\n
///                                       - done:    Number of bytes already erased.\n
///                                       - totsize: Total number of bytes to be erased.\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue erasing\n
///                                          - FALSE: Stop erasing and exit UDLDoUpdate with the following error codes:\n
///                                                   UDL_USER_ABORT when formatting/unloading or UDL_FLASH_ERASE_ERROR when erasing
/// @param[in]        SectionInfo:        Function pointer to a callback function which is called before updating\n
///                                       a new section returning information about the section.\n
///                                       Set this parameter to NULL if not required.\n
///                                       BOOL SectionInfo(DWORD SectionStart, DWORD SectionLen, DWORD SectionCRC)\n
///                                       - SectionStart: Start of the current section\n
///                                       - SectionLen:   Length of the current section\n
///                                       - SectionCRC:   CRC of the current section\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue update process\n
///                                          - FALSE: Stop update process and exit UDLDoUpdate (UDL_USER_ABORT)
/// @param[in]        WriteProgress       Function pointer to a callback function which is called during\n
///                                       writing to the flash. Set this parameter to NULL if not required.\n
///                                       BOOL WriteProgress(DWORD writtenSectionLen, DWORD writtenTotalLen)\n
///                                       - writtenSectionLen: Number of bytes of the current section which are already\n
///                                                            written to the flash.\n
///                                       - writtenTotalLen:   Total number of bytes which are already\n
///                                                            written to the flash.\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue update process\n
///                                          - FALSE: Stop update process and exit UDLDoUpdate (UDL_USER_ABORT)
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_INVALID_FILE_PATH
/// @retval            UDL_INVALID_BIN_FILE
/// @retval            UDL_INVALID_FILE_VERSION
/// @retval            UDL_MEM_ALLOC_FAILED
/// @retval            UDL_FLASH_ERASE_ERROR
/// @retval            UDL_FLASH_WRITE_ERROR
/// @retval            UDL_FLASH_READ_ERROR
/// @retval            UDL_USER_ABORT
/// @retval            UDL_WRITE_VERIFICATION_FAILED
/// @retval            UDL_FILE_SYSTEM_ERROR
/// @retval            UDL_FILE_SYSTEM_DRIVER_LOAD_ERROR
/// @retval            UDL_FILE_SYSTEM_FORMATING_ERROR
/// @retval            UDL_CPLD_NOT_FOUND
/// @retval            UDL_CPLD_READ_WRITE_ERROR
UDL_RET_TYPE UDLDoUpdate( PUDL_CALBACK_PROC_2 EraseProgress, PUDL_CALBACK_PROC_3 SectionInfo, PUDL_CALBACK_PROC_2 WriteProgress);

//******************************************************************************
/// Erases the specified regions in Flash.
/// @param[in]        regID               Defines the regions to delete. Regions don't have to be contiguous.
/// @param[in]        EraseProgress       Function pointer to a callback function which is called during\n
///                                       erasing of the flash.\n
///                                       The totsize is replied for every single region separately.\n
///                                       Set this parameter to NULL if not required.\n
///                                       BOOL EraseProgress(DWORD done, DWORD totsize)\n
///                                       - done:    Number of bytes already erased.\n
///                                       - totsize: Total number of bytes to be erased.\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue erasing\n
///                                          - FALSE: Stop erasing and exit UDLEraseRegion (UDL_FLASH_ERASE_ERROR)\n
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_REGION_NOT_FOUND
/// @retval            UDL_FLASH_ERASE_ERROR
///
/// Remarks:          Take care when using this function. This function performs a Flash erase action, not
///                   a formatting action! E.g. ConfigBlock also loses MAC address info etc. 
UDL_RET_TYPE UDLEraseRegion(UDL_REGION_ID regID, PUDL_CALBACK_PROC_2 EraseProgress);

//******************************************************************************
/// Reboots the system.
/// @param[in]        clean       - TRUE:  Perform a coldboot (clean reboot).\n
///                               - FALSE: Perform a warmboot.
/// @retval            UDL_SUCCESS        (but this would never happen)
UDL_RET_TYPE UDLReboot(BOOL clean);

//******************************************************************************
/// Save the registry to flash (RegFlush).
/// Return Value Type: UDL_RET_TYPE:
/// @retval            UDL_SUCCESS
/// @retval            UDL_ERROR
UDL_RET_TYPE UDLSaveRegistry(void);

//******************************************************************************
/// Clear the registry in flash.
/// @retval            UDL_SUCCESS
/// @retval            UDL_ERROR
///
/// Remarks:          The registry in flash is cleared. This doesn't change the current registry in a running system.
///                   The effect of the cleared flash registry shows up after a coldboot only.
UDL_RET_TYPE UDLClearRegistry(void);

//******************************************************************************
/// Writes the content of an nb0 file to flash.
/// @param[in]        pNB0Path:           Pointer to a string containing the path of the nb0 file
/// @param[in]        SectionInfo:        Function pointer to a callback function which is called before updating\n
///                                       a new section returning information about the section.\n
///                                       Set this parameter to NULL if not required.\n
///                                       BOOL SectionInfo(DWORD SectionStart, DWORD SectionLen, DWORD SectionCRC)\n
///                                       - SectionStart: Start of the current section\n
///                                       - SectionLen:   Length of the current section\n
///                                       - SectionCRC:   CRC of the current section\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue update process\n
///                                          - FALSE: Stop update process and exit UDLDoUpdate (UDL_USER_ABORT)
/// @param[in]        WriteProgress       Function pointer to a callback function which is called during\n
///                                       writing to the flash. Set this parameter to NULL if not required.\n
///                                       BOOL WriteProgress(DWORD writtenSectionLen, DWORD writtenTotalLen)\n
///                                       - writtenSectionLen: Number of bytes of the current section which are already\n
///                                                            written to the flash.\n
///                                       - writtenTotalLen:   Total number of bytes which are already\n
///                                                            written to the flash.\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue update process\n
///                                          - FALSE: Stop update process and exit UDLDoUpdate (UDL_USER_ABORT)
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_INVALID_FILE_PATH
/// @retval            UDL_INVALID_BIN_FILE
/// @retval            UDL_MEM_ALLOC_FAILED
/// @retval            UDL_FLASH_INIT_ERROR
/// @retval            UDL_FLASH_WRITE_ERROR
/// @retval            UDL_FLASH_READ_ERROR
/// @retval            UDL_CRC_ERROR
/// @retval            UDL_USER_ABORT
/// @retval            UDL_INSUFFICIENT_MEMORY
UDL_RET_TYPE UDLTegraFlashUpdate(TCHAR *pNB0Path, PUDL_CALBACK_PROC_3 SectionInfo, PUDL_CALBACK_PROC_2 WriteProgress);


//******************************************************************************
/// Completly wipes the flash of a tegra module including flash meta information.
//  Wipe of the flash is similar as done in flashing the module with recovery mode. 
/// @retval            UDL_SUCCESS
/// @retval            UDL_FLASH_ERASE_ERROR
/// @retval            UDL_FLASH_INIT_ERROR
/// @retval            UDL_HW_NOT_SUPPORTED
/// @retval            UDL_DISMOUNT_ERROR
UDL_RET_TYPE UDLTegraWipeFlashDisk();


//******************************************************************************
/// Writes the content of a SplashScreen bmz file to flash.
/// @param[in]        pSSPath:            Pointer to a string containing the path of the bmz file
/// @param[in]        SectionInfo:        Function pointer to a callback function which is called before updating\n
///                                       a new section returning information about the section.\n
///                                       Set this parameter to NULL if not required.\n
///                                       BOOL SectionInfo(DWORD SectionStart, DWORD SectionLen, DWORD SectionCRC)\n
///                                       - SectionStart: Start of the current section\n
///                                       - SectionLen:   Length of the current section\n
///                                       - SectionCRC:   CRC of the current section\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue update process\n
///                                          - FALSE: Stop update process and exit UDLDoUpdate (UDL_USER_ABORT)
/// @param[in]        WriteProgress       Function pointer to a callback function which is called during\n
///                                       writing to the flash. Set this parameter to NULL if not required.\n
///                                       BOOL WriteProgress(DWORD writtenSectionLen, DWORD writtenTotalLen)\n
///                                       - writtenSectionLen: Number of bytes of the current section which are already\n
///                                                            written to the flash.\n
///                                       - writtenTotalLen:   Total number of bytes which are already\n
///                                                            written to the flash.\n
///                                       - Return Value:\n
///                                          - TRUE:  Continue update process\n
///                                          - FALSE: Stop update process and exit UDLDoUpdate (UDL_USER_ABORT)
/// @retval            UDL_SUCCESS
/// @retval            UDL_NOT_INITIALIZED
/// @retval            UDL_INVALID_FILE_PATH
/// @retval            UDL_FLASH_WRITE_ERROR
/// @retval            UDL_USER_ABORT
/// @retval            UDL_ERROR
UDL_RET_TYPE UDLPxaSSUpdate(TCHAR *pSSPath, PUDL_CALBACK_PROC_3 SectionInfo, PUDL_CALBACK_PROC_2 WriteProgress);

#ifdef __cplusplus
}
#endif

#endif // _UPDATELIB_H_

