/* =======================================================================================================
  STCSharedData.c

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
 ------------------------------------------------------------------------------


======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <errno.h> 		// Error integer and strerror() function
#include <time.h>		// to get time for data log

#include <sys/mman.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>	// queue
#include <sys/shm.h>	// shared memory
#include <sys/sem.h>	// semaphore

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCShData.h"

static int  MaxErrMesgs = 10;

static bool IPCInit		= false;

static int  IPCSysShMID	= -1;
static int  IPCSysSemID	= -1;

static int  IPCLogQueID	= -1;
static int  IPCSLQueID	= -1;
static int  IPCCANQueID	= -1;

static STCSYS *pSysData;					// The STC data

static struct sembuf shlock   = {0,-1, 0},	// lock/unlock sem 0
			  	     shunlock = {0, 1, 0};

static struct timespec InitTime;


/* -------------------------------------------------------------------------------------------------------
   Shared area and Mutex for GPS data
   - create/refer shared area for gps data
   - attach the sh to local private structure
   - create/refer the semaphore (mutex) to control the access to the shared memory
   return:
    0	ShM and sem successfully created
   -1	some error (printed on stdout)
------------------------------------------------------------------------------------------------------- */
static int initsysarea()
{
	int sid = -1,
		mid = -1,
		R = 5;
	static struct shmid_ds iMem;

	// -------------------------------------------------------------------
	// 1. Create/access the shared memory dedicated to ST System data
	// -------------------------------------------------------------------
    mid = shmget(SYSDATASHMKEY,sizeof(STCSYS),IPC_CREAT | IPC_EXCL | O_RDWR | 0666);
	if(mid == -1) {						// error
		if(errno == EEXIST) { 			// because shm already exist
		    mid = shmget(SYSDATASHMKEY,sizeof(STCSYS),0);
			if(mid == -1) {				// error again
				printf("STCShData E!. ShM 2nd access error [%d]\n",errno);
				return(-1);
			}
		}
		else {							// for some other reason
			printf("STCShData E!. ShM 1st access error [%d]\n",errno);
			return(-1);
		}
    }

	// Get ShM info
	if(mid > 0) {
	    R = shmctl(mid,IPC_STAT,&iMem);
		if(R < 0) {						// error
			printf("STCShData E!. ShM control error [%d]\n",errno);
			return(-1);
		}
	}

	//printf("initsysarea(): sh = %d\n",mid);
	IPCSysShMID = mid;

	// -------------------------------------------------------------------
	// 2. Create/refer the semaphore to control the SM access
	// -------------------------------------------------------------------
	int retry = 10;
	SEMUN arg;

	arg.val = 1;						// 0 = lock; 1 = unlock

	sid = semget(SYSDATASEMKEY,1,IPC_CREAT | IPC_EXCL | O_RDWR | 0666 );
	if(sid == -1) {						// error
		if(errno == EEXIST) { 			// because Sem already exist
			sid = semget(SYSDATASEMKEY,1,0);
			if(sid == -1) {
				printf("STCShData E!. SEM 2nd access error [%d]\n",errno);
				return(-1);
			}
			// if the locker died from some reason, the semaphore could be
			//  locked and system will deadlock
			// a possible solution is to access the semaphore with a NOWAIT
			//  flag for N times, then force unlock
			shlock.sem_flg = IPC_NOWAIT;
			do {
				//R=semctl(sid,0,GETVAL,arg);		// which is the state?
				R=semop(sid,&shlock,1);			// prova a bloccare
				if(R == 0)	{ 					// e riuscito a boccare
					R=semop(sid,&shunlock,1);	// sblocca ed esce
					break;
				}
				usleep(10*retry);				// 100 => 10 microsecs
			}while(--retry);
			if(R < 0) {							// dead-lock
				R = semctl(sid, 0, SETVAL, arg);// force reset
				if(R < 0) {
					printf("STCShData E!. SEM reset error [%d]\n",errno);
					return(-1);
				}
			}
			shlock.sem_flg = 0;
		}
		else {							// for some other reason
			printf("STCShData E!. SEM 1st access error [%d]\n",errno);
			return(-1);
		}
    }
	else {									// created with no error
		R = semctl(sid, 0, SETVAL, arg);	// must be initialized by creator
		if(R < 0) {
			printf("STCShData E!. SEM initialization error [%d]\n",errno);
			return(-1);
		}
	}

	//printf("initsysarea(): sem = %d\n",sid);
	IPCSysSemID = sid;

	// -------------------------------------------------------------------
	// 3. Map the Shared memory to local (private) structure
	// -------------------------------------------------------------------
	pSysData = shmat(mid,NULL,0);
	if(pSysData == NULL) {
		printf("STCShData E!. SM attach error [%d]\n",errno);
		return(-1);
	}

	return(0);
}


/* -------------------------------------------------------------------------------------------------------
  Queue for error log
   Error data will be enqueued and later managed by a low priority application.
   - create/access the error queue
   return:
    0	queue successfully created
   -1	some error (printed on stdout)
------------------------------------------------------------------------------------------------------- */
static int initerrqueue()
{
	int		qid = -1;
	UINT	iValue = 0,
			ErrQueSize = 16384;  // default in /proc/sys/kernel/msgmnb
	FILE	*pQs;
	char	sValue[16];

	// -------------------------------------------------------------------
	pQs=fopen("/proc/sys/kernel/msgmnb","r");
	if(pQs != NULL) {
		if(fgets(sValue,16,pQs) != NULL)
			iValue = atoi(sValue);
		fclose(pQs);
	}
	if(iValue > 0) {
		ErrQueSize = iValue / 2;
		MaxErrMesgs = (ErrQueSize / sizeof(LOGMSG));
	}
	// -------------------------------------------------------------------
	qid = msgget(LOGDATAQUEKEY,IPC_CREAT | IPC_EXCL | O_RDWR | 0666);
	if(qid == -1) {						// error
		if(errno == EEXIST) { 			// because que already exist
		    qid = msgget(LOGDATAQUEKEY,0);
			if(qid == -1) {				// error again
				printf("STCShData E!. Queue 2nd access error [%d]\n",errno);
				return(-1);
			}
		}
		else {							// for some other reason
			printf("STCShData E!. Queue 1st access error [%d]\n",errno);
			return(-1);
		}
    }

	// ---------------------------------------------------------------
	// Read the status of the queue
	// ---------------------------------------------------------------
	//struct msqid_ds iQue;
	//msgctl(queid,IPC_STAT,&iQue);

	// ---------------------------------------------------------------
	// Initialize the system time
	// ---------------------------------------------------------------
	timespec_get(&InitTime,TIME_UTC);

	IPCLogQueID = qid;
	return(0);
}

/* -------------------------------------------------------------------------------------------------------
  Queue for SL commands
   The Listener send commands to the SL manager via SL queue
   return:
    0	queue successfully created
   -1	some error (printed on stdout)
------------------------------------------------------------------------------------------------------- */
static int initslqueue()
{
	int		qid = -1;
	UINT	iValue = 0,
			ErrQueSize = 16384;  // default in /proc/sys/kernel/msgmnb
	FILE	*pQs;
	char	sValue[16];

	// -------------------------------------------------------------------
	pQs=fopen("/proc/sys/kernel/msgmnb","r");
	if(pQs != NULL) {
		if(fgets(sValue,16,pQs) != NULL)
			iValue = atoi(sValue);
		fclose(pQs);
	}
	if(iValue > 0) {
		ErrQueSize = iValue / 2;
		MaxErrMesgs = (ErrQueSize / sizeof(LOGMSG));
	}
	// -------------------------------------------------------------------
	qid = msgget(SLCMDQUEKEY,IPC_CREAT | IPC_EXCL | O_RDWR | 0666);
	if(qid == -1) {						// error
		if(errno == EEXIST) { 			// because que already exist
		    qid = msgget(SLCMDQUEKEY,0);
			if(qid == -1) {				// error again
				printf("STCShData E!. Queue 2nd access error [%d]\n",errno);
				return(-1);
			}
		}
		else {							// for some other reason
			printf("STCShData E!. Queue 1st access error [%d]\n",errno);
			return(-1);
		}
    }

	// ---------------------------------------------------------------
	// Read the status of the queue
	// ---------------------------------------------------------------
	//struct msqid_ds iQue;
	//msgctl(queid,IPC_STAT,&iQue);

	IPCSLQueID = qid;
	return(0);
}

/* -------------------------------------------------------------------------------------------------------
  Queue for CAN commands
   The Listener send commands to the CAN manager via CAN queue
   return:
    0	queue successfully created
   -1	some error (printed on stdout)
------------------------------------------------------------------------------------------------------- */
static int initcanqueue()
{
	int		qid = -1;
	UINT	iValue = 0,
			ErrQueSize = 16384;  // default in /proc/sys/kernel/msgmnb
	FILE	*pQs;
	char	sValue[16];

	// -------------------------------------------------------------------
	pQs=fopen("/proc/sys/kernel/msgmnb","r");
	if(pQs != NULL) {
		if(fgets(sValue,16,pQs) != NULL)
			iValue = atoi(sValue);
		fclose(pQs);
	}
	if(iValue > 0) {
		ErrQueSize = iValue / 2;
		MaxErrMesgs = (ErrQueSize / sizeof(LOGMSG));
	}
	// -------------------------------------------------------------------
	qid = msgget(CANCMDQUEKEY,IPC_CREAT | IPC_EXCL | O_RDWR | 0666);
	if(qid == -1) {						// error
		if(errno == EEXIST) { 			// because que already exist
		    qid = msgget(CANCMDQUEKEY,0);
			if(qid == -1) {				// error again
				printf("STCShData E!. Queue 2nd access error [%d]\n",errno);
				return(-1);
			}
		}
		else {							// for some other reason
			printf("STCShData E!. Queue 1st access error [%d]\n",errno);
			return(-1);
		}
    }

	// ---------------------------------------------------------------
	// Read the status of the queue
	// ---------------------------------------------------------------
	//struct msqid_ds iQue;
	//msgctl(queid,IPC_STAT,&iQue);

	IPCCANQueID = qid;
	return(0);
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------ */
int IPCGetLogQueID()
{
	return(IPCLogQueID);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------ */
int IPCGetSLQueID()
{
	return(IPCSLQueID);
}

/* -------------------------------------------------------------------------------------------------------
	/proc/sys/kernel/msgmnb defines the max size of the queue (default = 16384); then
	the queue can accept N/sizeof(LOGMSG) messages.
	Append the log message (as a structure) to the queue. STCLogger will download and record.
------------------------------------------------------------------------------------------------------ */
void IPCLogS(LOGMSG *tMsg)
{
	LOGMSG	waste;
	struct msqid_ds iQue;
    struct timespec ts;
    struct tm *tminfo;
	int    H,M,S,m;

	// -------------------------------------------------------------------
    clock_gettime(CLOCK_REALTIME,&ts);	// Get current time with nanosecond precision
    tminfo = localtime(&ts.tv_sec);		// Convert seconds to local time
    m = ts.tv_nsec/1e6;					// Extract milliseconds from nanoseconds
    S = tminfo->tm_sec;					// Seconds
    M = tminfo->tm_min;					// Minutes
    H = tminfo->tm_hour;				// Hours
	sprintf(tMsg->LogTime,"%02d%02d%02d.%03d",H,M,S,m);

	// -------------------------------------------------------------------
	msgctl(IPCLogQueID,IPC_STAT,&iQue);
	if(iQue.msg_qnum >= MaxErrMesgs) {						// too many msgs in queue ?
		do{													// discharge the queue
			msgrcv(IPCLogQueID,&waste,sizeof(LOGMSG),0,0);	// download the oldest (waste)
			msgctl(IPCLogQueID,IPC_STAT,&iQue);				// and check again
		}while(iQue.msg_qnum >= MaxErrMesgs);				// until no room
		printf("STCShData:IPCLogS W!. Error queue is full. Check STCLogger status\n");
	}
	if(msgsnd(IPCLogQueID,tMsg,sizeof(LOGMSG),IPC_NOWAIT) < 0) {
		printf("STCShData:IPCLogS W!. Error queue send error [%d]\n",errno);
	}
}

/* -------------------------------------------------------------------------------------------------------
	/proc/sys/kernel/msgmnb defines the max size of the queue (default = 16384); then
	the queue can accept N/sizeof(LOGMSG) messages.
	Convert data to Log structure and send the message to the queue. STCLogger will download and record.
------------------------------------------------------------------------------------------------------ */
void IPCLogP(STCAPPID appid,LOGLEV level,UINT code,char *msg)
{
	LOGMSG	tMsg;

	if(strlen(msg)>112)
		msg[111]='\0';

	// Should never happen
	if(appid < 11)
		appid = 99;

	// Should never happen
	if(level < CRITICAL || level > INFOLOG)
		level = 0;

	tMsg.LogLev	= level;
	tMsg.LogApp	= appid;
	tMsg.LogErrCode = code;
	strcpy(tMsg.LogMsg,msg);
	IPCLogS(&tMsg);
}

/* -------------------------------------------------------------------------------------------------------
   Initialize all the IPCs.
   All application should run this function at startup, in order to access global data and resources at
     OS level
   return:
    0	all initialization successfully performed
   -1	some error
------------------------------------------------------------------------------------------------------- */
int IPCInitialize()
{
	// -------------------------------------------------------------------
	// Already initialized
	// -------------------------------------------------------------------
	if(IPCInit == true)
		return(0);

	// -------------------------------------------------------------------
	// 1. IPC for system data
	// -------------------------------------------------------------------
	if(initsysarea() != 0) {
		printf("STCShData E!. sys SM error\n");
		return(-1);
	}

	// -------------------------------------------------------------------
	// 2. IPC for Error management
	// -------------------------------------------------------------------
	if(initerrqueue() != 0) {
		printf("STCShData E!. log queue error\n");
		return(-1);
	}

	// -------------------------------------------------------------------
	// 3. IPC for SL data
	// -------------------------------------------------------------------
	if(initslqueue() != 0) {
		printf("STCShData E!. SL queue error\n");
		return(-1);
	}

	// -------------------------------------------------------------------
	// 4. IPC for CAN data
	// -------------------------------------------------------------------
	if(initcanqueue() != 0) {
		printf("STCShData E!. CAN queue error\n");
		return(-1);
	}

	IPCInit = true;
	return(0);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void InitSTCShData()
{
	if(!semop(IPCSysSemID,&shlock,1)) {

		//sprintf(pSysData.appid,"STCControl v%d.%d.%c (build %04d)",
		//		VERSION,RELEASE,DRAFT,BUILDNUMBER);
		pSysData->SysState 	 	= INIT;
		pSysData->SysMode		= UNDEF;
		pSysData->SessionTime	= 0;
		pSysData->TotWorkTime	= 0;
		pSysData->EnableMoving	= false;

		pSysData->UDPCom.LstnSd			= -1;		// Listener socket
		pSysData->UDPCom.TalkSd  		= -1;		// Talker socket
		pSysData->UDPCom.STUDPPort 		= 9914;		// The ST UDPlistening port
		pSysData->UDPCom.CUsockfd  		= -1;		// Sockte file
		pSysData->UDPCom.CUUDPPort		= 9966;		// The other UDPlistening port
		pSysData->UDPCom.CUIsConnected 	= FALSE;	// The other is connected/recognized?

		pSysData->SLData.LevelingState = FALSE;		// the state of Self-leveling
		pSysData->SLData.FWVersion = 'x';			// the FW version (LTG firmware)
		pSysData->SLData.XAxis = 0.0f;				// Current X inclination
		pSysData->SLData.XAxis = 0.0f;				// Current Y inclination
		for(int i=0;i<3;i++) {
			pSysData->SLData.MotorState[i] = FALSE;	// State of the motors
			pSysData->SLData.Errors[i] = 0x00;		// Error bytes
		}
		semop(IPCSysSemID,&shunlock,1);
	}
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int  IPCGetError()
{
	return(0);
}

/* -------------------------------------------------------------------------------------------------------
	if(!semop(IPCSysSemID,&shlock,1)) {
		semop(IPCSysSemID,&shunlock,1);
	}
------------------------------------------------------------------------------------------------------- */
void SysSetState(STCSTATE p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		pSysData->SysState = p;
		semop(IPCSysSemID,&shunlock,1);
	}
}

STCSTATE SysGetState(void)
{
	STCSTATE res = INIT;
	if(!semop(IPCSysSemID,&shlock,1)) {
		res = pSysData->SysState;
		semop(IPCSysSemID,&shunlock,1);
	}
	return(res);
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SysSetMode(STCMODE p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		pSysData->SysMode = p;
		semop(IPCSysSemID,&shunlock,1);
	}
}
STCMODE SysGetMode(void)
{
	STCMODE res = UNDEF;
	if(!semop(IPCSysSemID,&shlock,1)) {
		res = pSysData->SysMode;
		semop(IPCSysSemID,&shunlock,1);
	}
	return(res);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
bool SysGetCUConnected()
{
	bool res = false;
	if(!semop(IPCSysSemID,&shlock,1)) {
		res = pSysData->UDPCom.CUIsConnected;
		semop(IPCSysSemID,&shunlock,1);
	}
	return(res);
}
void SysSetCUConnected(bool p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		pSysData->UDPCom.CUIsConnected = p;
		semop(IPCSysSemID,&shunlock,1);
	}
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SysSetWrkTimers(ULONG S,ULONG T)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		pSysData->SessionTime = S;
		pSysData->TotWorkTime = T;
		semop(IPCSysSemID,&shunlock,1);
	}
}
void SysGetWrkTimers(ULONG *S,ULONG *T)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		S = &pSysData->SessionTime;
		T = &pSysData->TotWorkTime;
		semop(IPCSysSemID,&shunlock,1);
	}
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
bool SysMovingEnabled()
{
	bool res = false;
	if(!semop(IPCSysSemID,&shlock,1)) {
		res = pSysData->EnableMoving;
		semop(IPCSysSemID,&shunlock,1);
	}
	return(res);
}
void SysEnableMoving(bool p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		pSysData->EnableMoving = p;
		semop(IPCSysSemID,&shunlock,1);
	}
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SysSetSLData(SLDATA *p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		memcpy(&pSysData->SLData,p,sizeof(SLDATA));
		semop(IPCSysSemID,&shunlock,1);
	}
}
void SysGetSLData(SLDATA *p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		memcpy(p,&pSysData->SLData,sizeof(SLDATA));
		semop(IPCSysSemID,&shunlock,1);
	}
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SysSetPosData(GNSS *p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		memcpy(&pSysData->GPSData,p,sizeof(GNSS));
		semop(IPCSysSemID,&shunlock,1);
	}
}
void SysGetPosData(GNSS *p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		memcpy(p,&pSysData->GPSData,sizeof(GNSS));
		semop(IPCSysSemID,&shunlock,1);
	}
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SysSetEnvData(ENV *p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		memcpy(&pSysData->ENVData,p,sizeof(ENV));
		semop(IPCSysSemID,&shunlock,1);
	}
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SysGetEnvData(ENV *p)
{
	if(!semop(IPCSysSemID,&shlock,1)) {
		memcpy(p,&pSysData->ENVData,sizeof(ENV));
		semop(IPCSysSemID,&shunlock,1);
	}
}

