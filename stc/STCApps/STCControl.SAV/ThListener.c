/* =======================================================================================================
  ThListener.c
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
#include "STFunctions.h"

static LOGMSG   eLog;

JsonPair JsonPairs[MAX_PAIRS];
static char RcvBuffer[MAXJSONLEN],
			SndBuffer[MAXJSONLEN];
struct sockaddr_in clientaddr;
socklen_t len = sizeof(struct sockaddr_in);

static void	SendAck(int);

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void* ThListener(void* thr_data)
{
	int		 count,n,AckCode;
	int		 i,k;
	int 	 optval = 1;
	STCAPPID UserID = 0,
			 UserList[] = { STCCUNIT, STCGPSDATA, STCLOGGER,
					 	    STCCONTROLLER, STCDIAGNOSTIC, STCSIMULATOR };
	bool	 KnownUser = false,
			 ProtMatch = false;

	UINT  	 uiVal;

	//char  stVal[MAX_VAL_LEN];

	// -------------------------------------------------------------------
    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCCONTROLLER;
	sprintf(eLog.LogMsg,"ThListener thread started");
	IPCLogErrorP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);

	// -------------------------------------------------------------------
    // ST side address/port
	// -------------------------------------------------------------------
	udp.LstnSd = 0;
	udp.STUDPPort = STCCfg.sys.UDPPort;								// The ST will listen on this port

	bzero((char *)&udp.STSideAddr,len);
    udp.STSideAddr.sin_family		= AF_INET; 						// IPv4
    udp.STSideAddr.sin_addr.s_addr	= htonl(INADDR_ANY);			// any address
    udp.STSideAddr.sin_port			= htons(udp.STUDPPort);			// ST port

    udp.LstnSd = socket(AF_INET,SOCK_DGRAM,0);
	if(udp.LstnSd < 0) {
		sprintf(eLog.LogMsg,"listening socket failed");
		IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
        printf("- ThListener. E!. socket() error %d\n",errno);
		return(NULL);
	}
	setsockopt(udp.LstnSd,SOL_SOCKET,SO_REUSEADDR,
		   (const void *)&optval,sizeof(int));

	// -------------------------------------------------------------------
    // Bind the socket with any local address
	// -------------------------------------------------------------------
	if(bind(udp.LstnSd,(const struct sockaddr *)&udp.STSideAddr,len) < 0) {
		sprintf(eLog.LogMsg,"listening bind failed");
		IPCLogErrorP(eLog.LogApp,CRITICAL,NETERROR,eLog.LogMsg);
        printf("- ThListener. E!. bind() error %d\n",errno);
		return(NULL);
	}

	// -------------------------------------------------------------------
	printf("- ThListener.I!. Waiting on [%s=%s/%d]\n",
			udp.STNetCard,
			udp.STIPAddress,
			udp.STUDPPort);


	// -------------------------------------------------------------------
	// Some debug settings
	// -------------------------------------------------------------------
	bool 	NoShowAgain = false;
	STCMODE ActMode;

	// -------------------------------------------------------------------
	// Infinite loop waiting for CU commands:
	// ST replies to CU with Ack/Nack on the same port
	// -------------------------------------------------------------------
	while(TRUE) {
		n = recvfrom(udp.LstnSd,RcvBuffer,MAXJSONLEN,0,
			     	 (struct sockaddr *)&clientaddr,&len);
		RcvBuffer[n] = '\0';

		if(n <= 0) {
			sprintf(eLog.LogMsg,"Error receiving UDP data");
			IPCLogErrorP(eLog.LogApp,ERRLOG,NETERROR,eLog.LogMsg);
	        printf("- ThListener. E!. recvfrom() error %d\n",errno);
			continue;
		}

		// -------------------------------------------------------------------
	   	// Check the condition to trust the CU:
	   	// - The Sender AppID is known
	   	// - The sender address is that declared in the config file
		// -------------------------------------------------------------------
		strcpy(udp.CUIPAddress,
			   inet_ntop(AF_INET,&clientaddr.sin_addr,udp.CUIPAddress,len));
		//printf("- Received [%s] from [%s]\n", RcvBuffer,udp.CUIPAddress);
		printf("- Received [%s]\n", RcvBuffer);

		// -------------------------------------------------------------------
		// Check source IP
		// -------------------------------------------------------------------
		if(strcmp(udp.CUIPAddress,STCCfg.cu.Address) && !NoShowAgain) {
			printf("- ThListener. W!. Connection from unknown IP [%s].\n",udp.CUIPAddress);
			sprintf(eLog.LogMsg,"Connection from unknown IP [%s]",udp.CUIPAddress);
			IPCLogErrorP(eLog.LogApp,WARNLOG,PROTERR,eLog.LogMsg);
			if(STCCfg.test.AllowAnyAddr) {
				sprintf(eLog.LogMsg,"Unknown IP allowed by config");
				IPCLogErrorP(eLog.LogApp,INFOLOG,PROTERR,eLog.LogMsg);
				printf("- ThListener. I!. Unknown IP [%s] allowed by config!\n",STCCfg.cu.Address);
				NoShowAgain = true;
			}
			else {
				sprintf(eLog.LogMsg,"Unknown IP rejected");
				IPCLogErrorP(eLog.LogApp,WARNLOG,PROTERR,eLog.LogMsg);
				printf("- ThListener. W!. Unknown IP [%s] rejected!\n",STCCfg.cu.Address);
				continue;
			}
		}

		// -------------------------------------------------------------------
		// Check JSON syntax
		// NACK = 03 JSON_SYNTAX
		// -------------------------------------------------------------------
	    count = JSONParse(RcvBuffer,JsonPairs);
	    if(count < 0) {
			SendAck(JSON_SYNTAX);
			continue;
	    }

	    // -------------------------------------------------------------------
	    // Check Client/Protocol
		// -------------------------------------------------------------------
		if(!KnownUser || !ProtMatch) {
			for(i = 0; i < count; i++) {

				if(!strcmp(JsonPairs[i].key,"id")) {				// check other ID is trustworthy
					UserID = atoi(JsonPairs[i].value);
					for(k=0;k<MAX_USERS;k++) {
						if(UserID == UserList[k]) {
							KnownUser = true;						// This ID is known
							break;
						}
					}
				}
				else if(!strcmp(JsonPairs[i].key,"prot")) {					// check the protocol is the same
					if(!strcmp(JsonPairs[i].value,STCPROTOCOL))
						ProtMatch = true;
				}
			}
			if(!KnownUser) {
				SendAck(JSON_UNKCLIENT);
				continue;
			}
			if(!ProtMatch) {
				SendAck(JSON_PROTVERS);
				continue;
			}
		}

	    // -------------------------------------------------------------------
		// Now perform the request actions (if no error in the command)
		// NACK = 00 JSON_NOERR
		//	    = 04 JSON_CMDDATA
		// -------------------------------------------------------------------
		AckCode = JSON_NOERR;
		SendAck(JSON_NOERR);

	    for(i = 0; i < count; i++) {

			// Non params commands
			if(!strcmp(JsonPairs[i].key,"id")) {
			}
			else if(!strcmp(JsonPairs[i].key,"prot")) {
			}
			else if(!strcmp(JsonPairs[i].key,"cmd")) {
				// Enable regular telemetry (see ThTalker)
				if(!strcmp(JsonPairs[i].value,"SYNC")) {
					SysSetCUConnected(TRUE);
			        printf("- ThListener. I!. Enable regular telemetry\n");
				}
				else if(!strcmp(JsonPairs[i].value,"DELLOG")) {
					ActMode = SysGetMode();
					SysSetMode(BUSY);
					STDeleteLog(0);
					SysSetMode(ActMode);
				}
				else if(!strcmp(JsonPairs[i].value,"STORAGE")) {
					ActMode = SysGetMode();
					SysSetMode(BUSY);
					STStoragePosition();
					SysSetMode(ActMode);
				}
				else if(!strcmp(JsonPairs[i].value,"RESET")) {
					STResetAll();
				}
				else if(!strcmp(JsonPairs[i].value,"FACTORY")) {
					//STSetFactory()
					STReboot();
				}
				else if(!strcmp(JsonPairs[i].value,"SELFTEST")) {
					ActMode = SysGetMode();
					SysSetMode(BUSY);
					STSelfTest();
					SysSetMode(ActMode);
				}
				else if(!strcmp(JsonPairs[i].value,"SHUTDOWN")) {
					SysSetState(DOWN);
					SysSetMode(UNDEF);
					STShutDown();
					break;
				}
				else if(!strcmp(JsonPairs[i].value,"REBOOT")) {
					SysSetState(DOWN);
					SysSetMode(UNDEF);
					STReboot();
					break;
				}
				else {
					AckCode = JSON_CMDDATA;
				}
			}
			else if(!strcmp(JsonPairs[i].key,"LEVEL")) {}
			else if(!strcmp(JsonPairs[i].key,"DATAFREQ")) {
				uiVal = atoi(JsonPairs[i].value);
				if(uiVal>=10 && uiVal <= 3000)						// 10 mSecs - 3 Sec
					STCCfg.cu.UpdTime = uiVal;
				else
					AckCode = JSON_CMDDATA;
			}
			else if(!strcmp(JsonPairs[i].key,"rpt")) {
				if(!strcmp(JsonPairs[i].value,"CONFIG")) {
					AddEvent(RPTCONFIG);
				}
				else if(!strcmp(JsonPairs[i].value,"ERRORS")) {}
				else if(!strcmp(JsonPairs[i].value,"FULL")) {}
				else if(!strcmp(JsonPairs[i].value,"SWVER")) {
					AddEvent(RPTSWVER);
				}
				else if(!strcmp(JsonPairs[i].value,"WRKTIME")) {
					AddEvent(RPTWRKTIME);
				}
				else if(!strcmp(JsonPairs[i].value,"LOG")) {}
				else if(!strcmp(JsonPairs[i].value,"GPSDATA")) {
					AddEvent(RPTGPSDATA);
				}
				else {
					AckCode = JSON_CMDDATA;
				}
			}
			else {
				printf("Not valid: %s\n", JsonPairs[i].key);
				AckCode = JSON_CMDDATA;
			}

			// Ha già inviato ACK sulla sintassi e controllo credenziali
			// Se c'è un errore invia un NACK al termine dell'esecuzione
			// (NON DOVREBBE MAI ACCADERE)
			if(AckCode != JSON_NOERR) {
				SendAck(AckCode);
				break;
			}
	    }

		// -------------------------------------------------------
		// Reset the Time Counter as it received data from CU
		// -------------------------------------------------------
		TimeoutCounter = STCCfg.cu.Timeout;

	}
	printf("Shutdown requested\n");
	return(0);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SendAck(int code)
{
	int n;

    // -------------------------------------------------------------------
	JSONAck(code,SndBuffer);

    // -------------------------------------------------------------------
    // -------------------------------------------------------------------
	pthread_mutex_lock(&Netlock);
	n = sendto(udp.LstnSd,SndBuffer,strlen(SndBuffer), 0,
		   (struct sockaddr *)&clientaddr,len);
    pthread_mutex_unlock(&Netlock);

    // -------------------------------------------------------------------
	if(n < 0) {
		sprintf(eLog.LogMsg,"Error sending UDP data");
		IPCLogErrorP(eLog.LogApp,ERRLOG,NETERROR,eLog.LogMsg);
        printf("- ThListener. E!. . sendto() error %d\n",errno);
	}
	else {
		switch(code) {
			case JSON_NOERR:
				//printf("Replied [%s] to [%s]\n", SndBuffer,udp.CUIPAddress);
				break;
			case JSON_UNKCLIENT:
				sprintf(eLog.LogMsg,"Syntax error parsing JSON");
				IPCLogErrorP(eLog.LogApp,WARNLOG,PROTERR,eLog.LogMsg);
				printf("- ThListener. W!. JSON from unknown client.\n");
				break;
			case JSON_PROTVERS:
				sprintf(eLog.LogMsg,"ST protocol mismatch.");
				IPCLogErrorP(eLog.LogApp,WARNLOG,PROTERR,eLog.LogMsg);
				printf("- ThListener. W!. ST protocol mismatch.\n");
				break;
			case JSON_SYNTAX:
				sprintf(eLog.LogMsg,"Syntax error parsing JSON");
				IPCLogErrorP(eLog.LogApp,WARNLOG,PROTERR,eLog.LogMsg);
				printf("- ThListener. W!. Syntax error parsing JSON.\n");
				break;
			case JSON_CMDDATA:
				sprintf(eLog.LogMsg,"Unknown command / Value out of range");
				IPCLogErrorP(eLog.LogApp,WARNLOG,PROTERR,eLog.LogMsg);
				printf("- ThListener. W!. Unknown command / Value out of range.\n");
				break;
			case JSON_NOTALLOWED:
			case JSON_INSUFRES:
				break;
		}
	}
}

