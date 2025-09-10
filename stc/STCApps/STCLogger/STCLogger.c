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


======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>	// queue
#include <sys/shm.h>	// shared memory
#include <sys/sem.h>	// semaphore
#include <sys/stat.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"
#include "Globals.h"

#include "BCounter.h"
#include "STCShData.h"
#include "STCLogger.h"

#define INCLUDE_GLOBALS

static LOGMSG   eLog;

static FILE *pHF;
static char LogPath[256];

/* -------------------------------------------------------------------------------------------------------
   params:
   - cfg file
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	char   *whoami;
	char   cfgfile[64];

	bool   EnableLog = true;
	int	   Today = -1,
		   queid;
	LOGMSG logmsg;
	SYSCFG syscfg;
	pHF = NULL;

	// -------------------------------------------------------------------
    time_t nowtime;
    struct tm * timeinfo;
    time(&nowtime);
    timeinfo = localtime(&nowtime);
	Today = timeinfo->tm_mday;

    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCLOGGER;
	(whoami = strrchr(argv[0], '/')) ? ++whoami : (whoami = argv[0]);

	// -------------------------------------------------------------------
	// I P C - Initialization
	// -------------------------------------------------------------------
	if(IPCInitialize() < 0) {
    	printf("STCGPSData E!. IPC initialization error\nExit\n");
        exit(1);
	}

	// -------------------------------------------------------------------
    // Program initialization
	// -------------------------------------------------------------------
	eLog.LogLev = INFOLOG;
	eLog.LogErrCode = STCNOERROR;
	sprintf(eLog.LogMsg,"%s[%d] v%d.%d.%c (build %04d)",
			whoami,eLog.LogApp,
			VERSION,RELEASE,DRAFT,BUILDNUMBER);
	printf("%s\n",eLog.LogMsg);
	IPCLogS(&eLog);

	// -------------------------------------------------------------------
	// Work environment and initial configuration
	// -------------------------------------------------------------------
	CFGSetDefault();

	// ------------------------------------------------------------------
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
	printf("%s: I!.\t cfg file: [%s]\n",whoami,cfgfile);

	if(CFGReadFile(cfgfile)) {
		printf("%s: I!.\t Created the default cfg file\n",whoami);
	}

	// ===========================================================
	// Get current configuration
	// -----------------------------------------------------------
	CFGGetSyscfg(&syscfg);

	if(syscfg.LogLevel < 1) {		// level = 0 => disabled
		EnableLog = false;
    	printf("%s: I!.\t log is disabled by config\n",whoami);
	}
	else {
		if(TodayLogFile() != 0) {
			EnableLog = false;
			printf("%s: W!.\t log file [%s] error. Log disabled!\n",whoami,LogPath);
		}
		else {
			printf("%s: I!.\t log file: [%s] Log level: [%d]\n",
					whoami,LogPath,syscfg.LogLevel);
		}
	}


	// -------------------------------------------------------------------
	// I P C - Error queue
	// -------------------------------------------------------------------
	queid = IPCGetLogQueID();

	// -------------------------------------------------------------------
	// Main loop
	// -------------------------------------------------------------------
	do {
		// ---------------------------------------------------------------
		// Wait until some data on the queue; then download all data from
		//   the queue.
		// If log is enabled write data on log file
		// Only those messages having the error level over the threshold
		//  the threshold is specified in the config file.
		// ---------------------------------------------------------------
		while(msgrcv(queid,&logmsg,sizeof(LOGMSG),0,0) >= 0) {

#ifdef DBGSTCLOG
			printf("%s: I!.\t Received:\n%s\n%d\n%d\n%d\n%s\n\n",
					whoami,
					logmsg.LogTime,
					logmsg.LogLev,
					logmsg.LogApp,
					logmsg.LogErrCode,
					logmsg.LogMsg);
#endif

			// -----------------------------------------------------------
			// When log is enable, check the date change:
			// - if new day, create a new daily file
			//   (if file error, disable log)
			// -----------------------------------------------------------
			if(EnableLog) {
				time(&nowtime);
				timeinfo = localtime(&nowtime);

				if(timeinfo->tm_mday != Today) {
					Today = timeinfo->tm_mday;
					if(TodayLogFile() != 0) {
						EnableLog = FALSE;
						continue;
					}
				}

				if(logmsg.LogLev <= syscfg.LogLevel) {
					fprintf(pHF,"%s %d %2d %04d %s\n",
							logmsg.LogTime,
							logmsg.LogLev,
							logmsg.LogApp,
							logmsg.LogErrCode,
							logmsg.LogMsg);
					fflush(pHF);
				}
			}
		}

	}while(1);

	exit(0);
}

/* -------------------------------------------------------------------------------------------------------
   Manage the daily log file
------------------------------------------------------------------------------------------------------- */
int TodayLogFile()
{
	int 		Result = 0;
	time_t 		my_time;
	struct tm * timeinfo;

	// -------------------------------------------------------------------
	time(&my_time);
	timeinfo = localtime (&my_time);

	// -------------------------------------------------------------------
	// Close yesterday file (if open)
	// -------------------------------------------------------------------
	if(pHF != NULL)
		fclose(pHF);

	// -------------------------------------------------------------------
	// This is the new daily log file
	// -------------------------------------------------------------------
	bzero((char *)&LogPath,sizeof(LogPath));
	sprintf(LogPath,"%s/STC_%2d%02d%02d.log",
			LOGPATH,
			timeinfo->tm_year-100,
			timeinfo->tm_mon+1,
			timeinfo->tm_mday);

	// -------------------------------------------------------------------
	// Open a new daily file and add a first log record
	// -------------------------------------------------------------------
	pHF=fopen(LogPath,"a+");		// open new file
	if(pHF == NULL) {
		Result = 1;					// some error;
	}
	else {
		fprintf(pHF,"\n\n----------\n   UTC    Lv Ap Code Info\n");
		IPCLogP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);
	}

	return(Result);
}

/* -------------------------------------------------------------------------------------------------------
   Build the log file name
------------------------------------------------------------------------------------------------------- */
void TodayFileName(char *out)
{
	char todayname[128];
	time_t 		my_time;
	struct tm * timeinfo;

	// -------------------------------------------------------------------
	bzero((char *)&todayname,sizeof(todayname));
	time(&my_time);
	timeinfo = localtime(&my_time);

	sprintf(todayname,"%s/STC_%2d%02d%02d.log",
			LOGPATH,
			timeinfo->tm_year-100,
			timeinfo->tm_mon+1,
			timeinfo->tm_mday);
	strcpy(out,todayname);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
bool FuncFileExist(const char *fanme)
{
	struct stat buffer;
	return(stat (fanme,&buffer) == 0);
}

