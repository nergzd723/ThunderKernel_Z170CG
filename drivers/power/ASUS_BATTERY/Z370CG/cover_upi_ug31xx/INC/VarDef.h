/*
Copyright (C) <2015>  uPI semiconductor corp.
    This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.
    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. <http://www.gnu.org/licenses/>.
*/


/**
 * @file VarDef.h
 *
 *  VarDef.h define the global variables
 *
 *
 * @version $Revision: 31 $
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/

#ifndef __VARDEF_H__
#define __VARDEF_H__

#include "UpiDef.h"


///-------------------------------------------------------------------------------///
///  Register Declaration
///-------------------------------------------------------------------------------///


///-------------------------------------------------------------------------------///
///  Data flash SubClass #7 => MutualDriPtr
///-------------------------------------------------------------------------------///
EXTERN GaugeVarType      UpiGauge;
EXTERN GSbsType         *ReportSBS;
EXTERN GSbsCmdType      *SBSCmd;
EXTERN GSbsType         *SBS;
EXTERN GOtpDataType     *GOTP;
EXTERN GAdcDataType     *GADC;
EXTERN GlobalDFVarType  *GGB;
EXTERN GConfigDataType  *CFG;
EXTERN GMeasDataType    *CurMeas; ///< current measurement queue
EXTERN GMeasDataType    *PreMeas; ///< previous measurement queue
EXTERN GCapDataType     *CAP;
EXTERN GVCapDataType    *VCAP;
EXTERN GDWORD            ApCtlStatus;
EXTERN PollingTimeType  *PollTime;

#if defined(FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
EXTERN GSHORT            sKboHiBound;
EXTERN GSHORT            sKboLowBound;
EXTERN GSHORT            sKboExtHiBound;
EXTERN GSHORT            sKboExtLowBound;
EXTERN GSHORT            sKboCurr;
EXTERN GBYTE             GKboErrCode;

EXTERN GBYTE             GVerifyMeasErrCode;
EXTERN GSHORT            sVerifyMeasCurr;
EXTERN GWORD             wVerifyMeasVolt;
#endif ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
EXTERN unsigned char     FactoryGGBXFile[];

///-------------------------------------------------------------------------------///
/// Board Factor
///-------------------------------------------------------------------------------///
EXTERN CONST BoardFactorType BoardFactor[];

///-------------------------------------------------------------------------------///
/// Capacity Function
///-------------------------------------------------------------------------------///
EXTERN GU2 *CapTableTempPtr;
EXTERN GU2 *CapTableOcvSocPtr;
EXTERN GU1 *CapTableUpdateIdxPtr;

///-------------------------------------------------------------------------------///
/// Config Function
///-------------------------------------------------------------------------------///
EXTERN GConfigMapType *CfgMapTablePtr;

///-------------------------------------------------------------------------------///
/// SBS Command Function Pointer
///-------------------------------------------------------------------------------///
EXTERN GWORD (* const StdCmdTable[])(GVOID);
EXTERN GWORD (* const StdCmdTablePage2[])(GVOID);
EXTERN GWORD (* const StdCmdTablePage3[])(GVOID);

///-------------------------------------------------------------------------------///
/// Mutex
///-------------------------------------------------------------------------------///
#if defined(FEATURE_PLAT_WINDOWS)
EXTERN GVOID* UpiSbsMutex; 
#elif defined(FEATURE_PLAT_LINUX)
EXTERN struct mutex *UpiSbsMutex;
#else  ///< for FEATURE_PLAT_WINDOWS
EXTERN GVOID* UpiSbsMutex;
#endif ///< for FEATURE_PLAT_WINDOWS

#if defined(FEATURE_PLAT_WINDOWS)
EXTERN GVOID* UpiCapMutex;
#elif defined(FEATURE_PLAT_LINUX)
EXTERN struct mutex *UpiCapMutex;
#else  ///< for FEATURE_PLAT_WINDOWS
EXTERN GVOID* UpiCapMutex;
#endif ///< for FEATURE_PLAT_WINDOWS

#if defined(FEATURE_PLAT_WINDOWS)
EXTERN GVOID* UpiTimeMutex;
#elif defined(FEATURE_PLAT_LINUX)
EXTERN struct mutex *UpiTimeMutex;
#else  ///< for FEATURE_PLAT_WINDOWS
EXTERN GVOID* UpiTimeMutex;
#endif ///< for FEATURE_PLAT_WINDOWS

#if defined(FEATURE_PLAT_WINDOWS)
EXTERN GVOID* UpiI2CMutex; 
#elif defined(FEATURE_PLAT_LINUX)
EXTERN struct mutex *UpiI2CMutex;
#else  ///< for FEATURE_PLAT_WINDOWS
EXTERN GVOID* UpiI2CMutex;
#endif ///< for FEATURE_PLAT_WINDOWS

#if defined(FEATURE_PLAT_WINDOWS)
///-------------------------------------------------///
/// AP Input type
///-------------------------------------------------///
EXTERN GINT   iGaugeType;
EXTERN GDWORD dwMainLoopDelayTime;
EXTERN GOtpDataType   GInputOTP;

#endif ///< for FEATURE_PLAT_WINDOWS
///-------------------------------------------------------------------------------///
/// Debug log
///-------------------------------------------------------------------------------///
#ifdef  FEATURE_BUFFER_ENABLE
EXTERN GCHAR DebugLogBuf[];
#endif  ///< end of FEATURE_BUFFER_ENABLE
#if defined(FEATURE_PLAT_WINDOWS)
  EXTERN FILE* FPLogFile;
  EXTERN int   ILogFileidex; 
#endif ///< for FEATURE_PLAT_WINDOWS
/// ******** MAIN CODE  VARIABLE DEFINITION  ******* ////
#endif ///< __VARDEF_H__


