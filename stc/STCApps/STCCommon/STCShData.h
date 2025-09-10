/* =======================================================================================================
  STCSharedData.h

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
 ------------------------------------------------------------------------------


======================================================================================================= */
#ifndef SHDATA_H_
#define SHDATA_H_

/* -------------------------------------------------------------------------------------------------------
   The full state of the ST machine
------------------------------------------------------------------------------------------------------- */
typedef struct {
	char  	 appid[48];
	STCSTATE SysState;
	STCMODE  SysMode;
	SLDATA	 SLData;
	GNSS     GPSData;
	ENV		 ENVData;
	ULONG 	 SessionTime;
	ULONG 	 TotWorkTime;
	bool  	 EnableMoving;
	UDPCOM   UDPCom;
}STCSYS;

/* -------------------------------------------------------------------------------------------------------
   Internal commands between threads/processes
------------------------------------------------------------------------------------------------------- */
typedef struct _IPCIntCmd {
	BYTE cmddata[16];
}IPCCMD;

/* -------------------------------------------------------------------------------------------------------
   Access to the STC system shared data
------------------------------------------------------------------------------------------------------- */
// Access to Log global data
int  IPCGetLogQueID();
void IPCLogP(STCAPPID,LOGLEV,UINT,char*);
void IPCLogS(LOGMSG *);
int  IPCGetError();

int  IPCGetSLQueID();

// Initialize all IPCs
int  IPCInitialize();

// Initialize Shared data
void InitSTCShData();

/* -------------------------------------------------------------------------------------------------------
   Controlled access to System Shared data
------------------------------------------------------------------------------------------------------- */
// Connection
void SysSetCUConnected(bool);
bool SysGetCUConnected(void);

STCSTATE SysGetState(void);
void SysSetState(STCSTATE);
STCMODE SysGetMode(void);
void SysSetMode(STCMODE);


void SysSetSLData(SLDATA *);
void SysGetSLData(SLDATA *);

void SysSetPosData(GNSS *);
void SysGetPosData(GNSS *);

void SysSetEnvData(ENV *);
void SysGetEnvData(ENV *);

bool SysMovingEnabled(void);
void SysEnableMoving(bool);
void SysSetWrkTimers(ULONG,ULONG);
void SysGetWrkTimers(ULONG *,ULONG *);


#endif /* SHDATA_H_ */

