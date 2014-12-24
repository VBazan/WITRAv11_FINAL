///
/// @file	     Params.h
/// @copyright   Copyright (c) 2013 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1754 $ 
/// @date        $Date: 2013-08-12 11:39:59 +0200 (Mo, 12 Aug 2013) $
/// @brief	     Configuration parameters definitions
/// 
/// @target      PXAxxx, T20
/// 

#ifndef __PARAMS_LIB_H__
#define __PARAMS_LIB_H__

#ifdef __cplusplus
extern "C" {
#endif


#define TAG_PRE_INVALID		3
#define TAG_VALID			1
#define TAG_POST_INVALID	0

typedef struct {
	DWORD len	: 14; //LSB
	DWORD flags	: 2;	
	DWORD id	: 16; //MSB
} TAG, *PTAG;


BOOL FlashParamInit(void);
DWORD FlashParamGet(DWORD id, void* pBuf, DWORD maxlen);
DWORD FlashParamSet(DWORD id, void* pBuf, DWORD len);
DWORD FlashParamClear(DWORD id);
DWORD FlashParamGetAddr();

#define ID_MAC			0
#define ID_MULT			1
#define ID_IP			2
#define ID_DL			3
#define ID_SER			4
#define ID_LOC			5
#define ID_GPIO			6
#define ID_MSC			7
#define ID_HW			8
#define ID_DBG			9
#define ID_MBS			10
#define ID_SS			11
#define ID_PROD         12
#define ID_PEX          13
#define ID_KARG         14
#define ID_SD           15
// user defined IDs
#define ID_USER		0x8000


// ID_MAC
/////////////////////////////////////////
typedef struct {
	DWORD tid : 24; // toradex id part
	DWORD cid : 24; // colibri id part
} MACCFG;


// ID_MULT
/////////////////////////////////////////
typedef struct {
	DWORD l		:	 5;  // Core PLL Run-Mode-to-Oscillator Ratio     - All PXA's
	DWORD n		:	 4;  // Core PLL Turbo-Mode-to-Run-Mode Ration    - All PXA's
 	DWORD t		:	 1;  // Turbo Mode Selector                       - PXA270
	DWORD b		:	 1;  // Fast Bus Mode Selector                    - PXA270
	DWORD a		:	 1;  // Mem. Controller Clock Alternate Setting   - PXA270
	DWORD k1db2	:	 1;  // SDRAM Clock Pin 2 div by 2 Control/Status - PXA270
	DWORD k0db2	:	 1;  // Synch. Static Memory Clock Pin 0 Status   - PXA270
	DWORD k0db4	:	 1;  // Synch. Static Memory Clock Pin 0 Status   - PXA270
	DWORD ht	:	 1;  // Half Turbo Mode Selector                  - PXA270
    DWORD hss   :    2;  // System Bus Clock Frequency Select         - PXA3XX
    DWORD sflfs :    2;  // Int. Mem. Controller Frequency Select     - PXA3XX
    DWORD smcfs :    3;  // Static Mem. Controller Frequency Select   - PXA3XX
    DWORD dfclkdiv : 3;  // DF CLK Divider Value                      - PXA3XX
    DWORD dmcfs :    3;  // Dynamic Mem. Frequency Controller Status  - PXA3XX
    DWORD res	  :	 3;  // Reserved for future usage
} MULTCFG;


// ID_IP
/////////////////////////////////////////
#define IPCFG_F_DHCP	1
typedef struct {
	DWORD ip;
	DWORD subnet;
	DWORD flags;
} IPCFG;


// ID_DL
/////////////////////////////////////////
// dlcfg flags
#define DLCFG_F_DOWNLOAD	        0x0001
#define DLCFG_F_SILENT		        0x0002
#define DLCFG_F_DLTYPE              0x001C  // 3 bit (bit 2, 3 and 4)
#define DLCFG_F_DLTYPE_ETH			0x0000
#define DLCFG_F_DLTYPE_USB			0x0004
#define DLCFG_F_DLTYPE_SD			0x0008
#define DLCFG_F_DLGPIOPOLINV        0x0020
#define DLCFG_F_IMG_CRC_ENABLE      0x0040
#define DLCFG_F_DLONERROR           0x0080

// dlcfg.gpio bit 7 is gpio enable
#define DLCFG_GPIO_ENABLE           0x80

// dlcfg.imgCrcAction defines
#define DLCFG_ICA_WARNING           0
#define DLCFG_ICA_HALT              1
#define DLCFG_ICA_NEXT_TRY          2
typedef struct {
	BYTE  delay;
	BYTE  gpio;
	WORD  flags;
    BYTE  breakchar[4];
    DWORD imgCRCAction   : 4;
    DWORD res            :28;
} DLCFG;


// ID_SER
/////////////////////////////////////////
// sercfg flags
#define SERCFG_F_KEEP_ENABLED         0x00000001
typedef struct {
	DWORD portaddr;
	DWORD speed;
	DWORD flags; // parity, stopbits, 8/7 data bits (maybe in future)
} SERCFG;


// ID_LOC
/////////////////////////////////////////
typedef struct {
	DWORD imgstart;
	DWORD regstart;
	DWORD regsize;
	DWORD bl2start;
	DWORD res1;
	DWORD res2;
} LOCCFG;


// ID_GPIO
/////////////////////////////////////////
typedef struct {
	DWORD gpdr0; // 0..31
	DWORD gpdr1; // 32..63
	DWORD gpdr2; // 64..95
	DWORD gpdr3; // 96..127
	DWORD gplr0;
	DWORD gplr1;
	DWORD gplr2;
	DWORD gplr3;
    DWORD pData[10];
} GPIOCFG;

typedef struct {
	DWORD gafr0_l;
	DWORD gafr0_u;
	DWORD gafr1_l;
	DWORD gafr1_u;
	DWORD gafr2_l;
	DWORD gafr2_u;
	DWORD gafr3_l;
	DWORD gafr3_u;
} GPIOCFG_DATA_270;

typedef struct {
    DWORD mfppull_0     :2;
    DWORD mfppull_1     :2;
    DWORD mfppull_2     :2;
    DWORD mfppull_3     :2;
    DWORD mfppull_4     :2;
    DWORD mfppull_5     :2;
    DWORD mfppull_6     :2;
    DWORD mfppull_7     :2;
    DWORD mfppull_8     :2;
    DWORD mfppull_9     :2;
    DWORD mfppull_10    :2;
    DWORD mfppull_11    :2;
    DWORD mfppull_12    :2;
    DWORD mfppull_13    :2;
    DWORD mfppull_14    :2;
    DWORD mfppull_15    :2;
    DWORD mfppull_16    :2;
    DWORD mfppull_17    :2;
    DWORD mfppull_18    :2;
    DWORD mfppull_19    :2;
    DWORD mfppull_20    :2;
    DWORD mfppull_21    :2;
    DWORD mfppull_22    :2;
    DWORD mfppull_23    :2;
    DWORD mfppull_24    :2;
    DWORD mfppull_25    :2;
    DWORD mfppull_26    :2;
    DWORD mfppull_27    :2;
    DWORD mfppull_28    :2;
    DWORD mfppull_29    :2;
    DWORD mfppull_30    :2;
    DWORD mfppull_31    :2;
    DWORD mfppull_32    :2;
    DWORD mfppull_33    :2;
    DWORD mfppull_34    :2;
    DWORD mfppull_35    :2;
    DWORD mfppull_36    :2;
    DWORD mfppull_37    :2;
    DWORD mfppull_38    :2;
    DWORD mfppull_39    :2;
    DWORD mfppull_40    :2;
    DWORD mfppull_41    :2;
    DWORD mfppull_42    :2;
    DWORD mfppull_43    :2;
    DWORD mfppull_44    :2;
    DWORD mfppull_45    :2;
    DWORD mfppull_46    :2;
    DWORD mfppull_47    :2;
    DWORD mfppull_48    :2;
    DWORD mfppull_49    :2;
    DWORD mfppull_50    :2;
    DWORD mfppull_51    :2;
    DWORD mfppull_52    :2;
    DWORD mfppull_53    :2;
    DWORD mfppull_54    :2;
    DWORD mfppull_55    :2;
    DWORD mfppull_56    :2;
    DWORD mfppull_57    :2;
    DWORD mfppull_58    :2;
    DWORD mfppull_59    :2;
    DWORD mfppull_60    :2;
    DWORD mfppull_61    :2;
    DWORD mfppull_62    :2;
    DWORD mfppull_63    :2;
    DWORD mfppull_64    :2;
    DWORD mfppull_65    :2;
    DWORD mfppull_66    :2;
    DWORD mfppull_67    :2;
    DWORD mfppull_68    :2;
    DWORD mfppull_69    :2;
    DWORD mfppull_70    :2;
    DWORD mfppull_71    :2;
    DWORD mfppull_72    :2;
    DWORD mfppull_73    :2;
    DWORD mfppull_74    :2;
    DWORD mfppull_75    :2;
    DWORD mfppull_76    :2;
    DWORD mfppull_77    :2;
    DWORD mfppull_78    :2;
    DWORD mfppull_79    :2;
    DWORD mfppull_80    :2;
    DWORD mfppull_81    :2;
    DWORD mfppull_82    :2;
    DWORD mfppull_83    :2;
    DWORD mfppull_84    :2;
    DWORD mfppull_85    :2;
    DWORD mfppull_86    :2;
    DWORD mfppull_87    :2;
    DWORD mfppull_88    :2;
    DWORD mfppull_89    :2;
    DWORD mfppull_90    :2;
    DWORD mfppull_91    :2;
    DWORD mfppull_92    :2;
    DWORD mfppull_93    :2;
    DWORD mfppull_94    :2;
    DWORD mfppull_95    :2;
    DWORD mfppull_96    :2;
    DWORD mfppull_97    :2;
    DWORD mfppull_98    :2;
    DWORD mfppull_99    :2;
    DWORD mfppull_100   :2;
    DWORD mfppull_101   :2;
    DWORD mfppull_102   :2;
    DWORD mfppull_103   :2;
    DWORD mfppull_104   :2;
    DWORD mfppull_105   :2;
    DWORD mfppull_106   :2;
    DWORD mfppull_107   :2;
    DWORD mfppull_108   :2;
    DWORD mfppull_109   :2;
    DWORD mfppull_110   :2;
    DWORD mfppull_111   :2;
    DWORD mfppull_112   :2;
    DWORD mfppull_113   :2;
    DWORD mfppull_114   :2;
    DWORD mfppull_115   :2;
    DWORD mfppull_116   :2;
    DWORD mfppull_117   :2;
    DWORD mfppull_118   :2;
    DWORD mfppull_119   :2;
    DWORD mfppull_120   :2;
    DWORD mfppull_121   :2;
    DWORD mfppull_122   :2;
    DWORD mfppull_123   :2;
    DWORD mfppull_124   :2;
    DWORD mfppull_125   :2;
    DWORD mfppull_126   :2;
    DWORD mfppull_127   :2;
    DWORD mfppull_0_2   :2;
    DWORD mfppull_1_2   :2;
    DWORD mfppull_2_2   :2;
    DWORD mfppull_3_2   :2;
    DWORD mfppull_4_2   :2;
    DWORD mfppull_5_2   :2;
    DWORD mfppull_6_2   :2;
    DWORD mfppull_7_2   :2;
    DWORD mfppull_8_2   :2;
    DWORD mfppull_9_2   :2;
    DWORD mfppull_10_2  :2;
    DWORD mfppull_11_2  :2;
    DWORD mfppull_12_2  :2;
    DWORD mfppull_13_2  :2;
    DWORD mfppull_14_2  :2;
    DWORD mfppull_15_2  :2;
    DWORD mfppull_16_2  :2;
    DWORD mfppull_17_2  :2;
} GPIOCFG_DATA_3xx;


// ID_MSC
/////////////////////////////////////////
typedef struct {
	WORD readAssert;	// (RDF+1) is also the write assert time. NOTE: we count from CS assert to data ready to be safe
	WORD notUsed;
} NONBURSTTYPE;

typedef struct {
	WORD  readAssert;	// (RDF+1) NOTE: we count from CS assert to data ready to be safe
	WORD writeAssert;	// (RDN+1)	
} SRAMTYPE;

typedef struct {
	WORD firstAccess;	// (RDF+1) is also the write assert time. NOTE: we count from CS assert to data ready to be safe
	WORD  nextAccess;	// (RDN+1)	
} BURSTTYPE;

typedef struct {
	WORD readAssert;	// (RDF+1) is also write assert time
	WORD readDeassert;	// (RDN*2) is also write deassert time	
} VLIOTYPE;
typedef struct {
	WORD type;
	WORD buswidth; //0: 32 bit 1: 16 bit
	union {
		NONBURSTTYPE	flash;
		SRAMTYPE		sram;
		BURSTTYPE		burst;
		VLIOTYPE		vlio;		
	};
	WORD recovery; // (RRR*2+1)
	BYTE buffer;
	BYTE res1;
} MSCCFG;


// ID_HW
/////////////////////////////////////////
typedef struct {
	WORD VerMaj;
	WORD VerMin;
	WORD Variant;
    WORD ProdID;
} HWCFG;


// ID_DBG
/////////////////////////////////////////
#define DBGCFG_F_SERIAL_DEBUG	1
#define DBGCFG_F_REG_READ		2
#define DBGCFG_F_REG_WRITE		4
typedef struct {
	DWORD flags;
} DBGCFG;


// ID_MBS
/////////////////////////////////////////
typedef struct {
	DWORD bscntr0;
	DWORD bscntr1;
	DWORD bscntr2;
	DWORD bscntr3;
} MBSCFG;


// ID_SS
/////////////////////////////////////////
typedef struct {
	DWORD width : 16;
	DWORD height : 16;
    DWORD bpp : 8;
	DWORD ldds : 8;
	DWORD type : 4;
	DWORD color : 1;
	DWORD dual : 1;
	DWORD overlay : 1;
	DWORD dpc : 1;
	DWORD pcp : 1;
	DWORD oep : 1;
	DWORD hsp : 1;
	DWORD vsp : 1;
	DWORD bs : 4;
    DWORD pclk;
    DWORD hsw : 8;
	DWORD vsw : 8;
	DWORD blw : 16;
    DWORD elw : 16;
	DWORD bfw : 16;
    DWORD efw : 16;
	DWORD acb : 16;
    DWORD disp_gpio : 8;
	DWORD bl_gpio : 8;    
    DWORD dispondelay : 16;
    DWORD disp_pol : 1;
    DWORD bl_pol : 1;
    DWORD pcddiv : 1;
    DWORD res0 : 7; // init to 0
    DWORD res1 : 6; // init to 1
    DWORD res2 : 16;// init to 0
} LCDSETTINGS;

#define SSCFG_F_ENABLED		1
#define SSCFG_F_DBGINFO		2
typedef struct {
	DWORD	FileAddr;
	DWORD	FileSize;
	DWORD	flags;
	LCDSETTINGS	LS;	
    DWORD   FBAddr;
	DWORD	EdidSlaveAddr : 7;
	DWORD   EdidEnable    : 1;
	DWORD   res0          : 24; // init to 0
} SSCFG;


// ID_PROD
/////////////////////////////////////////
typedef struct {
    DWORD	TestDay:23;
    DWORD	Testing:1;
    DWORD	TestVer:8;
} PRODCFG;


// ID_PEX
/////////////////////////////////////////
typedef struct {
    DWORD	L2Enable:1;
	DWORD	MPEnable:1;
	DWORD	WDEnable:1;
    DWORD	res1:29;
	DWORD	MemMB:16;
	DWORD	CarveoutMB:16;
    DWORD	res3;
} PEXCFG;


// ID_KARG
/////////////////////////////////////////
typedef struct {
    char    Arg[256];
} KARGCFG;


// ID_SD
/////////////////////////////////////////
typedef struct {
    DWORD ClkGpio    :8;
    DWORD CmdGpio    :8;
    DWORD Dat0Gpio   :8;
    DWORD Dat1Gpio   :8;
    DWORD Dat2Gpio   :8;
    DWORD Dat3Gpio   :8;
    DWORD CDGpio     :8;
    DWORD CDPol      :1;
    DWORD Drive      :3;
    DWORD res          :4;
} SDCFG;

// ID_USER
/////////////////////////////////////////
typedef union {
	BYTE	bUserData[256];
	WORD	wUserData[128];
	DWORD	dwUserData[64];
} USERCFG;


#ifdef __cplusplus
}
#endif
#endif //__PARAMS_LIB_H__