///
/// @file	     ost.h
/// @copyright   Copyright (c) 2012 Toradex AG
/// @author	     $Author: gap $
/// @version     $Rev: 1364 $ 
/// @date        $Date: 2013-02-19 16:36:00 +0100 (Di, 19 Feb 2013) $
/// @brief	     OS Timer Structures
/// 
/// @target      PXAxxx, T20
/// 

#ifndef _OST_H_
#define _OST_H_

#include "CommonReg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef volatile struct 
{
    DWORD    osmr0;  //OS timer match register 0
    DWORD    osmr1;  //OS timer match register 1
    DWORD    osmr2;  //OS timer match register 2
    DWORD    osmr3;  //OS timer match register 3
    DWORD    oscr0;  //OS timer counter register 0(compatible)
    DWORD    ossr;   //OS timer status register
    DWORD    ower;   //OS timer watchdog enable register
    DWORD    oier;   //OS timer interrupt enable register
    DWORD    osnr;   //OS timer snapshot register
    DWORD    reserved1[7];
    DWORD    oscr4;	 //OS timer counter register 4
    DWORD    oscr5;	 //OS timer counter register  5
    DWORD    oscr6;	 //OS timer counter register  6
    DWORD    oscr7;	 //OS timer counter register  7
    DWORD    oscr8;	 //OS timer counter register  8
    DWORD    oscr9;	 //OS timer counter register  9
    DWORD    oscr10; //OS timer counter register  10
    DWORD    oscr11;		//OS timer counter register  11
    DWORD    reserved2[8];
    DWORD    osmr4;  //OS timer match register 4
    DWORD    osmr5;	 //OS timer match register 5
    DWORD    osmr6;	 //OS timer match register 6
    DWORD    osmr7;  //OS timer match register 7
    DWORD    osmr8;  //OS timer match register 8
    DWORD    osmr9;  //OS timer match register 9
    DWORD    osmr10; //OS timer match register 10
    DWORD    osmr11; //OS timer match register 11
    DWORD    reserved3[8];
    DWORD    omcr4;  //OS timer match control register 4
    DWORD    omcr5;  //OS timer match control register 5
    DWORD    omcr6;  //OS timer match control register 6
    DWORD    omcr7;  //OS timer match control register 7
    DWORD    omcr8;  //OS timer match control register 8
    DWORD    omcr9;  //OS timer match control register 9
    DWORD    omcr10; //OS timer match control register 10
    DWORD    omcr11; //OS timer match control register 11
}OSTREG;

#define OST_BASE (PERIPH_REG_BASE + OST_OFFSET)

#define OST_TICKS_MS 3250  // 1ms in ticks (3.25x10^6tick/sec * 1/1000sec/msec)
#define OST_TICKS_US    3  // 1usec in ticks (3.25x10^6tick/sec * 1/1000000sec/usec)

#ifdef __cplusplus
}
#endif

#endif