///
/// @file        MapRegLib.h
/// @copyright   Copyright (c) 2013 Toradex AG
/// $Author: kia $
/// $Rev: 2293 $ 
/// $Date: 2014-05-27 16:40:57 +0200 (Di, 27 Mai 2014) $
/// @brief       Library to support Register Mapping 
/// 
/// @target      Colibri PXAxxx, T20 & T30, Apalis T30
/// 

#ifndef _MAPREG_H_
#define _MAPREG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MRL_VER_MAJ         1
#define MRL_VER_MIN         3
#define MRL_VER_BUILD       0

#define PAGE_SIZE     4096

#define UnMapMemory UnMapRegister

/// Struct for physical and virtual addresses - useful for DMA operations)
typedef struct {
    DWORD pa;
    DWORD va;
} MEM_T;

// Common Windows API not defined in EVC++
LPVOID AllocPhysMem(DWORD cbSize, DWORD fdwProtect, DWORD dwAlignmentMask, DWORD dwFlags, PULONG pPhysicalAddress);
BOOL FreePhysMem(LPVOID lpvAddress);
BOOL VirtualCopy(LPVOID lpvDest, LPVOID lpvSrc, DWORD cbSize, DWORD fdwProtect);
BOOL VirtualSetAttributes(LPVOID lpvAddress, DWORD cbSize, DWORD dwNewFlags, DWORD dwMask, LPDWORD lpdwOldFlags);
LPVOID CreateStaticMapping(DWORD dwPhysBase, DWORD dwSize);

//******************************************************************************
/// Map Registers at a physical address (map areas up to the PAGE_SIZE).
/// @param[in]            pa          Physical address             
/// @retval               Virtual address           
void *MapRegister(DWORD pa);

//******************************************************************************
/// Same as MapRegister() but can also map larger sections
/// @param[in]            pa              Physical address.
/// @param[in]            size            Size to map in bytes.         
/// @retval               Virtual address            
void *MapMemory(DWORD pa, DWORD size);

//******************************************************************************
/// Unmap Register
/// @param[in]          pRegs:      Pointer to virual address           
void UnMapRegister(volatile void *pRegs);

//******************************************************************************
/// Win CE 6.0 and 5.0 compatible version of AllocPhysMem()
/// @param[in]            cbSize              Number of bytes to allocate              
/// @param[in]            fdwProtect          Specifies the protection desired
/// @param[in]            dwAlignmentmask     Bit mask describing the byte alignment
/// @param[in]            dwFlags             Reserved, set to 0
/// @param[in]            pPhysicalAddress    Physical address
/// @retval               0                   Error
/// @retval               >0                  Virtual address
LPVOID AllocPhysMemCompat(DWORD cbSize, DWORD fdwProtect, DWORD dwAlignmentMask, DWORD dwFlags, PULONG pPhysicalAddress);

//******************************************************************************
/// Win CE 6.0 and 5.0 compatible version of FreePhysMem()
/// @param[in]            lpvAddress          Virtual Address                     
/// @retval               FALSE: Failure, TRUE: Success
BOOL FreePhysMemCompat(LPVOID lpvAddress);

//******************************************************************************
/// Win CE 6.0 and 5.0 compatible version of VirtualCopy()
/// @param[in]            lpvDest             Destination Virtual Address              
/// @param[in]            lpvSrc              Source Virtual Address
/// @param[in]            cbSize              Number of bytes to copy 
/// @param[in]            fdwProtect          Specifies the protection desired
/// @retval               FALSE: Failure, TRUE: Success
BOOL VirtualCopyCompat(LPVOID lpvDest, LPVOID lpvSrc, DWORD cbSize, DWORD fdwProtect);

//******************************************************************************
/// Win CE 6.0 and 5.0 compatible version of LockPages()\n
/// Locks into memory the specified region of the virtual address space
/// @param[in]            lpvAddress          Virtual Starting Address              
/// @param[in]            cbSize              Number of bytes to lock
/// @param[in]            pPFNs               Array of physical frame numbers
/// @param[in]            fOptions            Option Flags
/// @retval               FALSE: Failure, TRUE: Success
BOOL LockPagesCompat(LPVOID lpvAddress, DWORD cbSize, PDWORD pPFNs, int fOptions);

//******************************************************************************
/// Win CE 6.0 and 5.0 compatible version of VirtualSetAttributes()\n
/// Change the per-page attributes for a range of virtual memory
/// @param[in]            lpvAddress          Virtual Starting Address                          
/// @param[in]            cbSize              Number of bytes 
/// @param[in]            dwNewFlags          The new flags values              
/// @param[in]            dwMask              Mask the bits to be changed
/// @param[out]           lpdwOldFlags        If not NULL, returns the original value of the first page entry      
/// @retval               FALSE: Failure, TRUE: Success
BOOL VirtualSetAttributesCompat(LPVOID lpvAddress, DWORD cbSize, DWORD dwNewFlags, DWORD dwMask, LPDWORD lpdwOldFlags);

//******************************************************************************
/// Get Virtual Address from Physical Address
/// @param[in]            pa                  Physical Address
/// @param[in]            cached              TRUE: Cached, FALSE: Not cached
/// @retval               Virtual Address
void* OALPAtoVA(UINT32 pa, BOOL cached);

//******************************************************************************
/// Returns the library Version.
/// @param[out]           pVerMaj             Returns the major version number. Set this parameter to NULL if not required.
/// @param[out]           pVerMin             Returns the minor version number. Set this parameter to NULL if not required.
/// @param[out]           pBuild              Returns the build number.         Set this parameter to NULL if not required.
void MRLGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif //_MAPREG_H_
