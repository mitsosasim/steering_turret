/* =======================================================================================================
  STCLogger.c

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine

 ------------------------------------------------------------------------------
   v. 1.0 - 01/12/2024 - Manage logs data STC apps
 ------------------------------------------------------------------------------
   Reference:

 ------------------------------------------------------------------------------

 - STC is an UDP server. The CU (Client) connect to the STC and start
   sending commands. The Server (STC) replies by acknowledging commands and
   sending telemetries.

======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <ifaddrs.h>
#include <netdb.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"

#include "BCounter.h"
#include "STCShData.h"
#include "STCControl.h"
#include "STCProtocol.h"

#define INCLUDE_GLOBALS
#include "Globals.h"

static LOGMSG   eLog;

/* -------------------------------------------------------------------------------------------------------
   params:
   - cfg file
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	char   *whoami;
	char   cfgfile[64];

	ULONG  SwTime = 0,		// Session work time
		   TwTime = 0;		// Total work time

	// -------------------------------------------------------------------
    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCCONTROLLER;
	(whoami = strrchr(argv[0], '/')) ? ++whoami : (whoami = argv[0]);

	// -------------------------------------------------------------------
	// I P C - Initialization
	// -------------------------------------------------------------------
	if(IPCInitialize() < 0) {
    	printf("STCControl E!.\t IPC initialization error\nExit\n");
        exit(1);
	}

	// -------------------------------------------------------------------
    // Program initialization
	// -------------------------------------------------------------------
    eLog.LogApp = STCCONTROLLER;
	eLog.LogLev = INFOLOG;
	eLog.LogErrCode = STCNOERROR;
	sprintf(eLog.LogMsg,"%s[%d] v%d.%d.%c (build %04d)",
			whoami,eLog.LogApp,
			VERSION,RELEASE,DRAFT,BUILDNUMBER);
	printf("%s\n",eLog.LogMsg);
	IPCLogErrorS(&eLog);

	// -------------------------------------------------------------------
	// Work environment and initial configuration
	// -------------------------------------------------------------------
	InitSTCShData();
	CFGSetDefault();
	InitLocalData();
	InitWorkTimers(&SwTime,&TwTime);

	// -------------------------------------------------------------------
	// Work with specific/default config file
	// -------------------------------------------------------------------
	sprintf(cfgfile,"%s/%s",CFGPATH,CFGFILE);	// default
	if(argc == 2) {
		strcpy(cfgfile,argv[1]);				// specific
		if(!FuncFileExist(cfgfile)) {
			printf("%s: E!.\t Error in cfg file [%s]\nExit\n",whoami,cfgfile);
			exit(1);
		}
	}

	sprintf(eLog.LogMsg,"%s[%d] %s",
			whoami,eLog.LogApp,cfgfile);
	IPCLogErrorS(&eLog);
	printf("%s: I!.\t cfg file: [%s]\n",whoami,cfgfile);

	if(CFGReadFile(cfgfile)) {
		printf("%s: I!.\t Created the default cfg file\n",whoami);
	}

	// ===========================================================
	// Get current configuration
	// -----------------------------------------------------------
	CFGGetSystem(&STCCfg);
	TimeoutCounter = STCCfg.cu.Timeout;

	// -----------------------------------------------------------
	// Initialize mutex on ST data access
	// -----------------------------------------------------------
    if(pthread_mutex_init(&Datalock, NULL) != 0) {
    	sprintf(eLog.LogMsg,"Data Mutex init has failed");
    	IPCLogErrorP(eLog.LogApp,CRITICAL,MEMERROR,eLog.LogMsg);
        return(-1);
    }
    EventPtr = -1;
    memset(EventList,0,16);

	// -----------------------------------------------------------
	// Initialize mutex on UDP network access
	// -----------------------------------------------------------
    if(pthread_mutex_init(&Netlock, NULL) != 0) {
    	sprintf(eLog.LogMsg,"UDP Mutex init has failed");
    	IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
        return(-1);
    }

	// -----------------------------------------------------------
	// Initialize mutex on DGIO access
	// -----------------------------------------------------------
    if(pthread_mutex_init(&DIOlock, NULL) != 0) {
    	sprintf(eLog.LogMsg,"DGIO Mutex init has failed");
    	IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
        return(-1);
    }

	// -------------------------------------------------------------------
	sprintf(eLog.LogMsg,"ThUDPServer thread started");
	IPCLogErrorP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);

	// -------------------------------------------------------------------
    // Creating socket file descriptor
	// -------------------------------------------------------------------
    memset(&udp,0,sizeof(UDPCOM));

    // -----------------------------------------------------------
	// UDP interface and JSON initialization
	// -----------------------------------------------------------
	if(GetIPAddressPort())
		printf("STCControl: E!.\t Netcard/IP error\nExit\n");

	// -----------------------------------------------------------
	// Create threads
	// -----------------------------------------------------------
    pthread_t thr[4];
    pthread_create(&(thr[0]),NULL,&ThListener,NULL);
    pthread_create(&(thr[1]),NULL,&ThTalker,NULL);
    pthread_create(&(thr[2]),NULL,&ThCANOperator,NULL);
    pthread_create(&(thr[3]),NULL,&ThSafety,NULL);

	// do somethig so that CANOper perform the self test

	// -----------------------------------------------------------
	// Periodic check
	// -----------------------------------------------------------
    int counters[1] = { 0 };
	while(true) {
		sleep(1);

		SwTime++;
		TwTime++;

		// -------------------------------------------------------
		// Work Timer (update every minute)
		// -------------------------------------------------------
		if(++counters[0] == 60) {
			SysSetWrkTimers(SwTime,TwTime);
			UpdateWorkTimers(SwTime,TwTime);
			counters[0] = 0;
		}

		// -------------------------------------------------------
		// Check Timeout
		// -------------------------------------------------------
		if(STCCfg.cu.Timeout > 0) {
			if(--TimeoutCounter == 0) {
				SysSetCUConnected(FALSE);
		    	sprintf(eLog.LogMsg,"Control Unit timeout");
		    	IPCLogErrorP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);
				printf("STCControl: I!.\t Control Unit is offline?\n");
			}
		}

	}

	return(0);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void InitLocalData()
{
	sprintf(STCSys.appid,"STCControl v%d.%d.%c (build %04d)",
			VERSION,RELEASE,DRAFT,BUILDNUMBER);

	pthread_mutex_lock(&Datalock);
	STCSys.SysState = INIT;
	STCSys.SysMode  = UNDEF;
	pthread_mutex_unlock(&Datalock);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void InitWorkTimers(ULONG *S,ULONG *T)
{
	FILE  *pTF;
	struct stat st;
	char   WorkTimeFile[128];
	char   record[64];
	ULONG  SwTime = 0,		// Session work Time
		   TwTime = 0;		// Total work time

	// -------------------------------------------------------------------
    time_t current_time;
    char*  FirstRun;

	// -----------------------------------------------------------
	sprintf(WorkTimeFile,"%s/%s",DATAPATH,TIMEFILE);
	if(stat(WorkTimeFile,&st) == -1) {
	    current_time = time(NULL);
		FirstRun = ctime(&current_time);
		pTF=fopen(WorkTimeFile,"w");
		fprintf(pTF,"STC Total Working Time (seconds)\n");
		fprintf(pTF,"First run was: %s\n",FirstRun);
		fprintf(pTF,"LastSession: 0\n");
		fprintf(pTF,"TotalTime:   0\n");
	}
	else {
		pTF=fopen(WorkTimeFile,"r+");
		fgets(record,64,pTF);
		fgets(record,64,pTF);
		fscanf(pTF,"%13s %lu",record,&SwTime);	// ignore
		fscanf(pTF,"%13s %lu",record,&TwTime);	// to increase
	}
	S = &SwTime;
	T = &TwTime;
	fclose(pTF);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void UpdateWorkTimers(ULONG Session,ULONG Total)
{
	FILE  *pTF;
	char   WorkTimeFile[128];
	char   record[64];

	// -----------------------------------------------------------
	sprintf(WorkTimeFile,"%s/%s",DATAPATH,TIMEFILE);
	pTF=fopen(WorkTimeFile,"r+");
	fgets(record,64,pTF);
	fgets(record,64,pTF);
	fprintf(pTF,"LastSession: %lu\n",Session);
	fprintf(pTF,"TotalTime:   %lu\n",Total);
	fclose(pTF);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int GetIPAddressPort()
{
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];

    if(getifaddrs(&ifaddr) == -1) {
		sprintf(eLog.LogMsg,"getifaddrs() failed");
		IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
		return(-1);
    }

	for(ifa = ifaddr;ifa != NULL;ifa = ifa->ifa_next) {
		family = ifa->ifa_addr->sa_family;
		if(family == AF_INET) {
			s = getnameinfo(ifa->ifa_addr,sizeof(struct sockaddr_in),
			host,NI_MAXHOST,NULL,0,NI_NUMERICHOST);
			if(s != 0) {
				sprintf(eLog.LogMsg,"getnameinfo() failed");
				IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
				return(-1);
			}
			if(!strcmp(host,"127.0.0.1"))
				continue;
			else {
				strcpy(udp.STNetCard,ifa->ifa_name);
				strcpy(udp.STIPAddress,host);	// This is the local address
			}
		}
    }

    return(0);
 }

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
bool FuncFileExist(const char *fanme)
{
	struct stat buffer;
	return(stat (fanme,&buffer) == 0);
}

/* -------------------------------------------------------------------------------------------------------
   a FIFO list of 16 events
------------------------------------------------------------------------------------------------------- */
void AddEvent(EVENT event)
{
	pthread_mutex_lock(&Datalock);
	if(EventPtr < 15) {				// the list is not full
		EventList[++EventPtr] = event;
	}
	else if(EventPtr >=15) {		// the list is full: the first will be lost
		for(int i=1;i<16;i++)
			EventList[i-1] = EventList[i];
		EventList[EventPtr=15] = event;
	}
	pthread_mutex_unlock(&Datalock);
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
bool GetEvent(EVENT *event)
{
	bool res = false;
	int i;
	pthread_mutex_lock(&Datalock);
	if(EventPtr >= 0) {
		*event = EventList[0];
		for(i=1;i<EventPtr;i++)
			EventList[i-1] = EventList[i];
		EventList[i-1]=0;
		EventPtr--;
		res = true;
	}
	pthread_mutex_unlock(&Datalock);
	return(res);
}
