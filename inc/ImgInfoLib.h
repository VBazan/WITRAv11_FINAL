///
/// @file        ImgInfoLib.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// $Author: gap $
/// $Rev: 1760 $ 
/// $Date: 2013-08-13 17:41:54 +0200 (Di, 13 Aug 2013) $
/// @brief       Library to interact with Image Information
/// 
/// @target      PXAxxx, Tegra
/// 

#ifndef __IMGINFOLIB_H__
#define __IMGINFOLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

/// Library Version
#define IMG_VER_MAJ     1
#define IMG_VER_MIN     4
#define IMG_VER_BUILD   0

typedef struct ROMHDR {
    ULONG   dllfirst;               // first DLL address
    ULONG   dlllast;                // last DLL address
    ULONG   physfirst;              // first physical address
    ULONG   physlast;               // highest physical address
    ULONG   nummods;                // number of TOCentry's
    ULONG   ulRAMStart;             // start of RAM
    ULONG   ulRAMFree;              // start of RAM free space
    ULONG   ulRAMEnd;               // end of RAM
    ULONG   ulCopyEntries;          // number of copy section entries
    ULONG   ulCopyOffset;           // offset to copy section
    ULONG   ulProfileLen;           // length of PROFentries RAM 
    ULONG   ulProfileOffset;        // offset to PROFentries
    ULONG   numfiles;               // number of FILES
    ULONG   ulKernelFlags;          // optional kernel flags from ROMFLAGS .bib config option
    ULONG   ulFSRamPercent;         // Percentage of RAM used for filesystem 
                                        // from FSRAMPERCENT .bib config option
                                        // byte 0 = #4K chunks/Mbyte of RAM for filesystem 0-2Mbytes 0-255
                                        // byte 1 = #4K chunks/Mbyte of RAM for filesystem 2-4Mbytes 0-255
                                        // byte 2 = #4K chunks/Mbyte of RAM for filesystem 4-6Mbytes 0-255
                                        // byte 3 = #4K chunks/Mbyte of RAM for filesystem > 6Mbytes 0-255

    ULONG   ulDrivglobStart;        // device driver global starting address
    ULONG   ulDrivglobLen;          // device driver global length
    USHORT  usCPUType;              // CPU (machine) Type
    USHORT  usMiscFlags;            // Miscellaneous flags
    PVOID   pExtensions;            // pointer to ROM Header extensions
    ULONG   ulTrackingStart;        // tracking memory starting address
    ULONG   ulTrackingLen;          // tracking memory ending address
} ROMHDR;

// followed by nummods <TOCentry>'s
typedef struct TOCentry {           // MODULE BIB section structure
    DWORD dwFileAttributes;
    FILETIME ftTime;
    DWORD nFileSize;
    LPSTR   lpszFileName;
    ULONG   ulE32Offset;            // Offset to E32 structure
    ULONG   ulO32Offset;            // Offset to O32 structure
    ULONG   ulLoadOffset;           // MODULE load buffer offset
} TOCentry, *LPTOCentry;

// followed by numfiles <TOCentry>'s
typedef struct FILESentry {         // FILES BIB section structure
    DWORD dwFileAttributes;
    FILETIME ftTime;
    DWORD nRealFileSize;
    DWORD nCompFileSize;
    LPSTR   lpszFileName;
    ULONG   ulLoadOffset;           // FILES load buffer offset
} FILESentry, *LPFILESentry;

typedef struct COPYentry {
    ULONG   ulSource;               // copy source address
    ULONG   ulDest;                 // copy destination address
    ULONG   ulCopyLen;              // copy length
    ULONG   ulDestLen;              // copy destination length 
                                    // (zero fill to end if > ulCopyLen)
} COPYentry;


#define VI_MAGIC1    0x01234567
#define VI_MAGIC2    0x89abcdef

typedef struct {
    DWORD Magic1;
    DWORD ID;
    DWORD VerMaj;
    DWORD VerMin; // upper 16 bit are build nr (for BETAs)
    DWORD Magic2;
} VERSIONINFO;

// BSP Version Structure 
typedef struct {
    DWORD ID;
    DWORD Maj;
    DWORD Min;
} BSPVER;

//******************************************************************************
/// Initialise the Image Information Library. 
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL ImgInfoLibInit(void);

//******************************************************************************
/// Frees the Image Information library
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL ImgInfoLibDeInit(void);

//******************************************************************************
/// Finds the BSP Version starting from 0x00 for 256Kb
/// @param[out]         pBspVer         Pointer to BSP Version Structure
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL FindVersionInfo(BSPVER *pBspVer);

//******************************************************************************
/// Finds the BSP Version, use the supplied start address and length
/// @param[in]          startAddr       Start Address              
/// @param[in]          length          Search length              
/// @param[out]         pBspVer         Pointer to BSP Version Structure
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL FindVersionInfoEx(BSPVER *pBspVer, DWORD startAddr, DWORD length);

//******************************************************************************
/// Finds the given string
/// @param[in]          pStr            Pointer to zero-terminated string to find              
/// @retval             >0              Pointer to last character of the string
/// @retval             NULL(0)         String not found
char *FindVersionString(char *pStr);

//******************************************************************************
/// Get the Table of Contents Pointer
/// @param[in]          address         Starting address              
/// @param[out]         pTOC            Pointer to TOC
/// @param[out]         pTOCOffset      Pointer to TOC Offset
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL GetTOC(DWORD address, ROMHDR *pTOC, DWORD *pTOCOffset);

//******************************************************************************
/// Get the Image Size
/// @param[in]          address        Starting Address
/// @retval             Image Size in bytes
///
/// Remarks:            We also search at 32KB offset for romhdr (eboot offset\n
///                     in eboot_formatted.bin)
DWORD GetIMAGESize(DWORD address);

//******************************************************************************
/// Find the image in the Flash
/// @param[in]          bImage          TRUE: Skip First Sector              
/// @param[out]         pStart          Pointer to start of image
/// @param[out]         pLen            Pointer to length of image
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL FindIMAGE(BOOL bImage, DWORD *pStart, DWORD *pLen);

//******************************************************************************
/// Get the Bootloader info from the flash, search the whole of the flash
/// @param[in]          pBspVer         Pointer to BSP Version              
/// @param[in]          dateStr         Date String              
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL GetFlashBootloaderInfo(BSPVER *pBspVer, TCHAR *dateStr);

//******************************************************************************
/// Get the Bootloader info from the flash
/// @param[in]          pBspVer         Pointer to BSP Version              
/// @param[in]          dateStr         Date String  
/// @param[in]          start           Start Address  
/// @param[in]          length          Length
/// @retval             TRUE            Success
/// @retval             FALSE           Error
BOOL GetFlashBootloaderInfoEx(BSPVER *pBspVer, TCHAR *dateStr, DWORD start, DWORD length);

//******************************************************************************
/// Get the Current Imae Info
/// @param[in]          pBspVer         Pointer to BSP Version              
/// @param[in]          dateStr         Date String  
void GetCurrentImageInfo(BSPVER *pBspVer, TCHAR *dateStr);

//******************************************************************************
/// Outputs library version
/// @param[out] verMaj  Major version number. Set this parameter to NULL if not required
/// @param[out] verMin  Minor version number. Set this parameter to NULL if not required
/// @param[out] build   Build number.         Set this parameter to NULL if not required
void IMGGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif // __IMGINFOLIB_H__
