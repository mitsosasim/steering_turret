/* =======================================================================================================
  ThTalker.c
  Created on: Feb 25, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 25, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <unistd.h>
#include <pthread.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"
#include "Globals.h"

#include "STCControl.h"
#include "STCProtocol.h"
#include "STCShData.h"
#include "BCounter.h"

static LOGMSG   eLog;
static char		SndBuffer[MAXJSONLEN];

#define TIMEUNIT 10000		// 10 mSec is the time unit

void SendData(void);
void ReportSWVER();
void ReportWRKTime();
void ReportConfig();
void ReportGPSData() ;

/*---------------------------------------------------------------------
---------------------------------------------------------------------*/
void* ThTalker(void* thr_data)
{
	int 	UpdateTime,TimeCounter = 0;				// in mSecs
	int 	optval = 1;
	EVENT   report;
	socklen_t len = sizeof(struct sockaddr_in);

	// -------------------------------------------------------------------
    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCCONTROLLER;
	sprintf(eLog.LogMsg,"ThTalker thread started");
	IPCLogErrorP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);

	// -------------------------------------------------------------------
    // CU side address/port
	// -------------------------------------------------------------------
	udp.TalkSd = 0;
	udp.CUUDPPort = STCCfg.cu.UDPPort;			// The CU will listen on this port
	strcpy(udp.CUIPAddress,STCCfg.cu.Address);	// (not known) This is the expected address

	bzero((char *)&udp.CUSideAddr,len);
    udp.CUSideAddr.sin_family		= AF_INET; 						// IPv4
    udp.CUSideAddr.sin_addr.s_addr 	= inet_addr(udp.CUIPAddress);
    udp.CUSideAddr.sin_port			= htons(udp.CUUDPPort);			// CU port

    UpdateTime = STCCfg.cu.UpdTime * 1000;  // from mSecs to uSecs

    udp.TalkSd = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	if(udp.TalkSd < 0) {
		sprintf(eLog.LogMsg,"Talker socket failed");
		IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
        printf("- ThTalker. E!.\t socket() error %d\n",errno);
		return(NULL);
	}
	setsockopt(udp.TalkSd,SOL_SOCKET,SO_REUSEADDR,
		   (const void *)&optval,sizeof(int));

	// -------------------------------------------------------------------
	printf("- ThTalker. I!.\t Telemetry on [%s:%d] every [%d] mSecs\n",
			udp.CUIPAddress,
			udp.CUUDPPort,
			STCCfg.cu.UpdTime);

	// -------------------------------------------------------------------
	while(true) {

		// -------------------------------------------------------------------
		// There is none to talk to, until some device will connect
		// -------------------------------------------------------------------
		while(!SysGetCUConnected())
			usleep(100);

		// -------------------------------------------------------------------
		// This is the address from where the Listener received data
		// -------------------------------------------------------------------
	    udp.CUSideAddr.sin_addr.s_addr 	= inet_addr(udp.CUIPAddress);

		// -------------------------------------------------------------------
		// -------------------------------------------------------------------
	    do {

			// ---------------------------------------------------------------
			// Prepare the telemetry
	    	// ---------------------------------------------------------------
	    	strcpy(SndBuffer,"Test STC telemetry!");

			// ---------------------------------------------------------------
		    // Send regular telemetry to the Control Unit
			// ---------------------------------------------------------------
			SendData();

		    // ---------------------------------------------------------------
			// 'UpdateTime' is the time between regular telemetry (master slot).
			// In the meanwhile if some request, this will be performed during
			//  the idle time (idle slot)
			// ---------------------------------------------------------------
		    UpdateTime = STCCfg.cu.UpdTime * 1000;  					// from mSecs to uSecs (it can change on the fly)
		    do {
		    	usleep(TIMEUNIT);
		    	TimeCounter += TIMEUNIT;
				if(GetEvent(&report)) {
			        printf("- ThTalker. I!.\t The event %d occurs\n",report);
			        switch(report) {
			        	case RPTSWVER:
			        		ReportSWVER();
			        		break;
			        	case RPTWRKTIME:
			        		ReportWRKTime();
			        		break;
			        	case RPTCONFIG:
			        		ReportConfig();
			        		break;
			        	case RPTGPSDATA:
			        		ReportGPSData();
			        	case RPTERRORS:
			        	case RPTFULL:
			        	case RPTLOG:
			        		break;
			        }
				}

		    }while(TimeCounter < UpdateTime);
		    TimeCounter = 0;

	    } while(udp.CUIsConnected);
	}

	return(0);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SendData()
{
	int 	N,L;
	socklen_t len = sizeof(struct sockaddr_in);

	L = strlen(SndBuffer);

	pthread_mutex_lock(&Netlock);
   	N = sendto(udp.TalkSd,
   				(const char *)SndBuffer,L,
				MSG_DONTWAIT,
				(const struct sockaddr *)&udp.CUSideAddr,
				len);
    pthread_mutex_unlock(&Netlock);

	// -------------------------------------------------------------------
    if(N > 0) {
    //    printf("ThTalker. I!.\t [%s] sent to [%s:%d]\n",
    //    		SndBuffer,udp.CUIPAddress,udp.CUUDPPort);
    }
    else {
		sprintf(eLog.LogMsg,"Talker sendto() failed");
		IPCLogErrorP(eLog.LogApp,ERRLOG,NETERROR,eLog.LogMsg);
        printf("- ThTalker. E!.\t sendto() error %d\n",errno);
    }
}


/* -------------------------------------------------------------------------------------------------------
   { "rpt": "GPSDATA",
     "nrec": 1,
     "trec": 1,
     "rec": "LAT": 41.889896, "LON": 12.491316, "ALT": 27.0, "HEAD": 0.12345678 }
------------------------------------------------------------------------------------------------------- */
void ReportGPSData() {
	char RecordContent[128];
	int  TotalRecords = 1,
		 CurrentRecord = 1;
	SMGNSS GPSData;

	pthread_mutex_lock(&Netlock);
	memcpy(&GPSData,&gps,sizeof(SMGNSS));
    pthread_mutex_unlock(&Netlock);

	// -------------------------------------------------------------------
	sprintf(RecordContent,"LAT=%0.8f LON=%0.8f ALT=%0.8f HEAD=%0.8f PITCH=%0.8f ROLL=%0.8f",
			GPSData.latitude,GPSData.longitude,GPSData.Altitude,
			GPSData.Heading,GPSData.Pitch,GPSData.Roll);
	sprintf(SndBuffer,"{ \"rpt\": \"GPSDATA\", \"nrec\": %d, \"trec\": %d,\"rec\": \"%s\" }",
			CurrentRecord,TotalRecords,RecordContent);
	SendData();
}

/* -------------------------------------------------------------------------------------------------------
   { "rpt": "SWVER",
     "nrec": 1,
     "trec": 1,
     "rec": "application SW version" }
------------------------------------------------------------------------------------------------------- */
void ReportSWVER()
{
	char RecordContent[48];
	int  TotalRecords = 1,
		 CurrentRecord = 1;

	// -------------------------------------------------------------------
	pthread_mutex_lock(&Datalock);
	pthread_mutex_unlock(&Datalock);

	sprintf(RecordContent,"STCControl v%d.%d.%c (build %04d)",
			VERSION,RELEASE,DRAFT,BUILDNUMBER);
	sprintf(SndBuffer,"{ \"rpt\": \"SWVER\", \"nrec\": %d, \"trec\": %d,\"rec\": \"%s\" }",
			CurrentRecord,TotalRecords,RecordContent);
	SendData();
}

/* -------------------------------------------------------------------------------------------------------
   { "rpt": "WRKTIME",
     "nrec": 1,
     "trec": 1,
     "rec": "time counters" }
------------------------------------------------------------------------------------------------------- */
void ReportWRKTime()
{
	char RecordContent[48];
	int  TotalRecords = 1,
		 CurrentRecord = 1;
	ULONG TotTime,
		  SesTime;
	int  Ht=0,Mt=0,St=0;
	int  Hs=0,Ms=0,Ss=0;

	// -------------------------------------------------------------------
	SysGetWrkTimers(&SesTime,&TotTime);

	// -------------------------------------------------------------------
	if(SesTime > 3600) {
		Hs = (int)(SesTime/3600);
		SesTime -= (Hs*3600);
	}
	if(SesTime>60) {
		Ms = (int)(SesTime/60);
		SesTime -= (Ms*60);
	}
	Ss = SesTime;

	// -------------------------------------------------------------------
	if(TotTime > 3600) {
		Ht = (int)(TotTime/3600);
		TotTime -= (Ht*3600);
	}
	if(TotTime>60) {
		Mt = (int)(TotTime/60);
		TotTime -= (Mt*60);
	}
	St = TotTime;

	// -------------------------------------------------------------------
	sprintf(RecordContent,"Last Session: %d:%02d:%02d, Total time: %d:%02d:%02d",
			Hs,Hs,Ss,Ht,Mt,St);
	sprintf(SndBuffer,"{ \"rpt\": \"WRKTIME\", \"nrec\": %d, \"trec\": %d,\"rec\": \"%s\" }",
			CurrentRecord,TotalRecords,RecordContent);
	SendData();
}

/* -------------------------------------------------------------------------------------------------------
   { "rpt": "CFGFILE",
     "nrec": 1,
     "trec": 1,
     "rec": "filename" }
   { "rpt": "CFGDATA",
     "nrec": 1,
     "trec": N,
     "rec": "record N" }
------------------------------------------------------------------------------------------------------- */
void ReportConfig()
{
	FILE  *pHc;
	char  cfgfile[64];
	char  record[128];
	int   L,TotalRecords = 0;

	// -------------------------------------------------------------------
	CFGGetCurFile(cfgfile);

	const char *filename = strrchr(cfgfile, '/');
	filename++;
	sprintf(SndBuffer,"{ \"rpt\": \"CFGFILE\", \"nrec\": %d, \"trec\": %d,\"rec\": \"%s\" }",
			1,1,filename);
	SendData();

	// -----------------------------------------------------------
	pHc=fopen(cfgfile,"r");
	while(fgets(record,128,pHc))
		TotalRecords++;
	fseek(pHc,0,SEEK_SET);
	for(int i=0;i<TotalRecords;i++) {
		fgets(record,128,pHc);
		L=strlen(record);
		if(L) record[--L]='\0';
		sprintf(SndBuffer,"{ \"rpt\": \"CFGDATA\", \"nrec\": %d, \"trec\": %d,\"rec\": \"%s\" }",
				i+1,TotalRecords,record);
		SendData();
	}
	fclose(pHc);
}

