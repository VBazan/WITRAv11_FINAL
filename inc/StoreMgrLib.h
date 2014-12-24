//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//
#ifndef __STORAGEMGRLIB__
#define __STORAGEMGRLIB__


#include <windows.h>
#include <storemgr.h>

#ifdef __cplusplus
extern "C" {
#endif

//Version Info
#define STMGR_VER_MAJ     2
#define STMGR_VER_MIN     0
#define STMGR_VER_BUILD   0

BOOL    IsStorageManagerRunningCompat();
// Storage Management API's
HANDLE  OpenStoreCompat(LPCTSTR szDeviceName);
BOOL    DismountStoreCompat(HANDLE hStore);
BOOL    FormatStoreCompat(HANDLE hStore);
HANDLE  FindFirstStoreCompat(PSTOREINFO pStoreInfo);
BOOL    FindNextStoreCompat(HANDLE hSearch, PSTOREINFO pStoreInfo);
BOOL    FindCloseStoreCompat(HANDLE hSearch);
BOOL    GetStoreInfoCompat(HANDLE hStore, PSTOREINFO pStoreInfo);
// Partition Management API's
BOOL    CreatePartitionCompat(HANDLE hStore, LPCTSTR szPartitionName, SECTORNUM snNumSectors);
BOOL    CreatePartitionExCompat(HANDLE hStore, LPCTSTR szPartitionName, BYTE bPartType, SECTORNUM snNumSectors);
BOOL    DeletePartitionCompat(HANDLE hStore, LPCTSTR szPartitionName);
HANDLE  OpenPartitionCompat( HANDLE hStore, LPCTSTR szPartitionName);
BOOL    MountPartitionCompat(HANDLE hPartition);
BOOL    DismountPartitionCompat(HANDLE hPartition);
BOOL    RenamePartitionCompat(HANDLE hPartition, LPCTSTR szNewName);
BOOL    SetPartitionAttributesCompat(HANDLE hPartition, DWORD dwAttrs);
BOOL    GetPartitionInfoCompat(HANDLE hPartition, PPARTINFO pPartInfo);
BOOL    FormatPartitionCompat(HANDLE hPartition);
BOOL    FormatPartitionExCompat(HANDLE hPartition, BYTE bPartType, BOOL bAuto);
HANDLE  FindFirstPartitionCompat(HANDLE hStore, PPARTINFO pPartInfo);
BOOL    FindNextPartitionCompat(HANDLE hSearch, PPARTINFO pPartInfo);
BOOL    FindClosePartitionCompat(HANDLE hSearch);

/*************************************
Description:      Returns the library Version

Parameter:
                  Out-Parameter:
                  - pVerMaj:          Returns the major version number. Set this parameter to NULL if not required.
                  - pVerMin:          Returns the minor version number. Set this parameter to NULL if not required.
                  - pBuild:           Returns the build number.         Set this parameter to NULL if not required.
*************************************/
void STMGRGetLibVersion(DWORD *pVerMaj, DWORD *pVerMin, DWORD *pBuild);

#ifdef __cplusplus
}
#endif

#endif