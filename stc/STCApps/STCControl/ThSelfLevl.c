/* =======================================================================================================
  ThSelfLevl.c
  Created on: Aug 19, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Aug 19, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - ALMA-STC-123-ICD - SW Interface Control Document (ICD)
    R2
 ------------------------------------------------------------------------------
  Functions and logics
  When requested, perform the Self Level
  When requested, perform the sub-system IBIT

======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>

#include <unistd.h>
#include <pthread.h>
#include <termios.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"
#include "Globals.h"

#include "STCControl.h"
#include "STCShData.h"

#define  MAXSLMSGLEN  16
#define  SLSTAT		0xF0
#define  SLREPORT	0xF3
#define  SLHEALTH   0xF6
#define  SLTIMEOUT   900		// 15 minutes
#define  RSCHARTOUT   30

//#define SLDEBUG

static SLDATA	SLData;
static LOGMSG   eLog;
static int 		hRSPort;
static char		SndSLBuffer[MAXSLMSGLEN];
static char		RcvSLBuffer[MAXSLMSGLEN];

int  SLSetPort(void);
bool SLDataSend(void);
int  SLDataRecv(void);
void SLClearBuffer(void);

/*---------------------------------------------------------------------
---------------------------------------------------------------------*/
void* ThSelfLevl(void* thr_data)
{
	int 	queid;
	IPCCMD  CUcmd;

	int 	res;
	int 	SLConnect = FALSE;
    time_t  StartTime,SLTime,
			ElapsedTime;
    BYTE	MSB,LSB;

    // -------------------------------------------------------------------
    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCCONTROLLER;
	sprintf(eLog.LogMsg,"ThSelfLevl thread started");
	IPCLogP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);

	// -------------------------------------------------------------------
	printf("- ThSelfLevl. I!. Self-Level management on %s,%ld,8,N,1\n",
			STCCfg.sl.RSPort,STCCfg.sl.RSBaudRate);

	// -------------------------------------------------------------------
	// I P C - SL queue
	// -------------------------------------------------------------------
	queid = IPCGetSLQueID();
	while(msgrcv(queid,&CUcmd,sizeof(IPCCMD),0,IPC_NOWAIT) > 0) { }

	// -------------------------------------------------------------------
	// Try to access the UART(4)
	// -------------------------------------------------------------------
	if(SLSetPort() == 0) {
		SLConnect = TRUE;
		SLClearBuffer();
	}

	// -------------------------------------------------------------------
	// Initialize SL data
	// -------------------------------------------------------------------
	SysGetSLData(&SLData);

	// -------------------------------------------------------------------
	// Main loop
	// Wait for a command from CU:
	// - Start: start SL
	// - Stop: stop SL
	// - Report: Perform a check (report request)
	// Every 60 seconds, performs a check (CBIT)
	// -------------------------------------------------------------------
	memset(SndSLBuffer,0,MAXSLMSGLEN);
	while(TRUE) {
		// ---------------------------------------------------------------
		// Wait until some command (from CU -> Listener) on the queue
		// ---------------------------------------------------------------
		while(msgrcv(queid,&CUcmd,sizeof(IPCCMD),0,IPC_NOWAIT) >= 0) {

#if SLDEBUG
			printf("- ThSelfLevl. I!. Cmd. on queue: ");
			for(int i=0;i<3;i++)
				printf("%02X ",CUcmd.cmddata[i]);
			printf("\n");
#endif

			if(!SLConnect) {
#if SLDEBUG
				printf("ThSelfLevl: I!. cmd ignored due to RS port error\n");
#endif
				continue;
			}

			// ---------------------------------------------------------------
		    // forward the command to the SL
			// ---------------------------------------------------------------
			SLClearBuffer();
			SndSLBuffer[0] = CUcmd.cmddata[0];
			SndSLBuffer[1] = CUcmd.cmddata[1];
			SndSLBuffer[2] = SndSLBuffer[0] ^ SndSLBuffer[1];
			if(!SLDataSend()) {
				sprintf(eLog.LogMsg,"Error writing on port [%s]",STCCfg.sl.RSPort);
				IPCLogP(eLog.LogApp,ERRLOG,RSWRITE,eLog.LogMsg);
			}

			// ---------------------------------------------------------------
			// Enters the Self Level Management
		    // Manages the Self Level data (and check if a other commands
	        //  arrives on queue)
			// ---------------------------------------------------------------
	        do {

	        	// gets data from SL until leveling is complete
	        	res = SLDataRecv();

	        	// When Self levelling is active check:
				if(SLData.LevelingState == TRUE) {
					time(&SLTime);
					ElapsedTime = (SLTime - StartTime);

					// if SL is not completed within 15 minutes, raise an error
					if(ElapsedTime > SLTIMEOUT) {
	        			SLData.LevelingState = FALSE;
			        	SysSetSLData(&SLData);
						sprintf(eLog.LogMsg,"Self-Level not completed within 15 minutes");
						IPCLogP(eLog.LogApp,CRITICAL,FILERROR,eLog.LogMsg);
//#if SLDEBUG
						printf("- ThSelfLevl. W!. Self-Level Incomplete\n");
//#endif
					}

					// if SL stop sending data for a while,  raise an error
					if(res == 0) {
						sprintf(eLog.LogMsg,"No data from SL - Leveling Incomplete");
						IPCLogP(eLog.LogApp,CRITICAL,FILERROR,eLog.LogMsg);
						SLData.LevelingState = FALSE;
						SysSetSLData(&SLData);
//#if SLDEBUG
						printf("- ThSelfLevl. W!. No data from SL - Leveling Incomplete\n");
//#endif
						break;
					}
				}

	        	// SL sent the updated state
	        	if(RcvSLBuffer[0] == SLSTAT) {
	        		if(RcvSLBuffer[1] == 0x01) {
	        			SLData.LevelingState = TRUE;
	        		    time(&StartTime);
//#if SLDEBUG
	    				printf("- ThSelfLevl: I!. Self-level is ON\n");
//#endif
	        		}
	        		if(RcvSLBuffer[1] == 0x02) {
	        			SLData.LevelingState = FALSE;
//#if SLDEBUG
	    				printf("- ThSelfLevl: I!. Self-level is  OFF\n");
//#endif
	        		}
	        	}
	        	// SL sent the report
	        	else if(RcvSLBuffer[0] == SLREPORT) {
					SLData.MotorState[0] = (RcvSLBuffer[2] & 0x80? TRUE: FALSE);
					SLData.MotorState[1] = (RcvSLBuffer[2] & 0x40? TRUE: FALSE);
					SLData.MotorState[2] = (RcvSLBuffer[2] & 0x20? TRUE: FALSE);
					SLData.MotorState[3] = (RcvSLBuffer[2] & 0x10? TRUE: FALSE);
					MSB = RcvSLBuffer[3];
					LSB = RcvSLBuffer[4];
					if(MSB & 0x80)
						MSB &= 0x7F;
					SLData.XAxis = (float)MSB + (float)(LSB)/100.0f;

					MSB = RcvSLBuffer[5];
					LSB = RcvSLBuffer[6];
					if(MSB & 0x80)
						MSB &= 0x7F;
					SLData.YAxis = (float)MSB + (float)(LSB)/100.0f;

	        		if(RcvSLBuffer[1] == 0x01) {
	        			//	printf("ThSelfLevl: I!. X = %5.2lf | %5.2lf",dbgi,dbgd);
	    				printf("- ThSelfLevl: I!. [X = %3.2lf  Y = %3.2lf]\n",SLData.XAxis,SLData.YAxis);
	        			SLData.LevelingState = TRUE;
	        		}
	        		if(RcvSLBuffer[1] == 0x02) {
	        			SLData.LevelingState = FALSE;
#if SLDEBUG
	    				printf("- ThSelfLevl: I!. Self-level is  OFF\n");
#endif
	        		}
	        	}
	        	// SL sent the health status
	        	else if(RcvSLBuffer[0] == SLHEALTH) {
					SLData.FWVersion = RcvSLBuffer[1];
					// reserved		 = RcvSLBuffer[2];
					for(int i=0;i<4;i++)
						SLData.Errors[i] = RcvSLBuffer[3+i];
#if SLDEBUG
    				printf("- ThSelfLevl: I!. SL health status\n");
#endif
	        	}

	        	// Update the system data
	        	SysSetSLData(&SLData);

	        	// Check Timeout
	        	if(SLData.LevelingState == TRUE) {
					time(&SLTime);
					ElapsedTime = (SLTime - StartTime);
					if(ElapsedTime > SLTIMEOUT) {
	        			SLData.LevelingState = FALSE;
			        	SysSetSLData(&SLData);
						sprintf(eLog.LogMsg,"Self-Level not completed within 15 minutes");
						IPCLogP(eLog.LogApp,CRITICAL,FILERROR,eLog.LogMsg);
						printf("- ThSelfLevl. W!. Self-Level Incomplete\n");
					}
	        	}

	        	// A command in the queue during processing?
	        	if(msgrcv(queid,&CUcmd,sizeof(IPCCMD),0,IPC_NOWAIT) >= 0) {
	    			SndSLBuffer[0] = CUcmd.cmddata[0];
	    			SndSLBuffer[1] = CUcmd.cmddata[1];
	    			SndSLBuffer[2] = SndSLBuffer[0] ^ SndSLBuffer[1];

	    			// During the Self-Leveling, only the stop will be accepted
	    			if(SLData.LevelingState == TRUE) {
	    				if(SndSLBuffer[1] == 0x02) {
#if SLDEBUG
	    					printf("- ThSelfLevl: I!. Command accepted\n");
#endif
							if(SLDataSend()) {
								sprintf(eLog.LogMsg,"Error writing on port [%s]",STCCfg.sl.RSPort);
								IPCLogP(eLog.LogApp,ERRLOG,RSWRITE,eLog.LogMsg);
							}
	    				}
#if SLDEBUG
	    				else
	       					printf("- ThSelfLevl: I!. Command rejected\n");
#endif
	    			}
	        	}

	        }while(SLData.LevelingState == TRUE);
		}
	}

	return(0);
}

/* -------------------------------------------------------------------------------------------------------
   Open and set the UART to the proper baudrate
------------------------------------------------------------------------------------------------------- */
int SLSetPort()
{
	int    result = 0;
	struct termios tty;

	// ------------------------------------------------------
	// Open the RS port
	// ------------------------------------------------------
	hRSPort = open(STCCfg.sl.RSPort,O_RDWR | O_NOCTTY);
	if(hRSPort < 0) {
		result = errno;
		sprintf(eLog.LogMsg,"Error [%d] opening port [%s]",errno,STCCfg.sl.RSPort);
		IPCLogP(eLog.LogApp,CRITICAL,FILERROR,eLog.LogMsg);
		printf("- ThSelfLevl. E!. SetSLPort(): open(%s) error %d\n",STCCfg.sl.RSPort,errno);
		return(result);
	}

	// ------------------------------------------------------
	// Get curret RS settings
	// ------------------------------------------------------
	if(tcgetattr(hRSPort,&tty) != 0) {
		result = errno;
		sprintf(eLog.LogMsg,"Error [%d] reading port [%s]",errno,STCCfg.sl.RSPort);
		IPCLogP(eLog.LogApp,CRITICAL,FILERROR,eLog.LogMsg);
		printf("- ThSelfLevl. E!. SetSLPort(): tcgetattr(%s) error %d\n",STCCfg.sl.RSPort,errno);
		return(result);
	}

	// ------------------------------------------------------
	if(STCCfg.sl.RSParity == 'N')
		tty.c_cflag &= ~PARENB; 	// Clear parity bit, disabling parity (most common)
	else
		tty.c_cflag |= PARENB;  	// Set parity bit, enabling parity

	if(STCCfg.sl.RSStopBit == 1)
		tty.c_cflag &= ~CSTOPB; 	// Clear stop field, only one stop bit used in communication (most common)
	else
		tty.c_cflag |= CSTOPB;  	// Set stop field, two stop bits used in communication

	tty.c_cflag &= ~CSIZE; 			// Clear all the size bits, then use one of the statements below
	if(STCCfg.sl.RSDataBit == 8)
		tty.c_cflag |= CS8; 		// 8 bits per byte (most common)
	else if(STCCfg.sl.RSDataBit == 7)
		tty.c_cflag |= CS7; 		// 7 bits per byte
	else if(STCCfg.sl.RSDataBit == 6)
		tty.c_cflag |= CS6; 		// 6 bits per byte
	else if(STCCfg.sl.RSDataBit == 5)
		tty.c_cflag |= CS5; 		// 5 bits per byte

	tty.c_cflag &= ~CRTSCTS; 		// Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL;	// Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; 			// Disable echo
	tty.c_lflag &= ~ECHOE; 			// Disable erasure
	tty.c_lflag &= ~ECHONL; 		// Disable new-line echo
	tty.c_lflag &= ~ISIG; 			// Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; 			// Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; 			// Prevent conversion of newline to carriage return/line feed
	tty.c_cc[VMIN] = 0;				// wait for (at least) N characters (0 = do not wait)
	tty.c_cc[VTIME] = 10;    		// Wait for up to 1s (10 deciseconds), returning as soon as any data is received.

	// ------------------------------------------------------
	// Set in/out baudrate by calling cfsetispeed() / cfsetospeed()
	// ------------------------------------------------------
	switch(STCCfg.sl.RSBaudRate) {
		case 110:
			cfsetispeed(&tty,B110);
			break;
		case 300:
			cfsetispeed(&tty,B300);
			break;
		case 1200:
			cfsetispeed(&tty,B1200);
			break;
		case 2400:
			cfsetispeed(&tty,B2400);
			break;
		case 4800:
			cfsetispeed(&tty,B4800);
			break;
		case 9600:
			cfsetispeed(&tty,B9600);
			break;
		case 19200:
			cfsetispeed(&tty,B19200);
			break;
		case 38400:
			cfsetispeed(&tty,B38400);
			break;
		case 57600:
			cfsetispeed(&tty,B57600);
			break;
		case 115200:
			cfsetispeed(&tty,B115200);
			break;
		case 230400:
			cfsetispeed(&tty,B230400);
			break;
		case 460800:
			cfsetispeed(&tty,B460800);
			break;
		case 500000:
			cfsetispeed(&tty,B500000);
			break;
		case 576000:
			cfsetispeed(&tty,B576000);
			break;
		case 921600:
			cfsetispeed(&tty,B921600);
			break;
	}

	// ------------------------------------------------------
	// Finally try setting the new configuration
	// errno = 5 means this is a wrong port; try another
	// ------------------------------------------------------
	if(tcsetattr(hRSPort,TCSANOW, &tty) != 0) {
		result = errno;
		close(hRSPort);
		sprintf(eLog.LogMsg,"Error [%d] setting port [%s]",errno,STCCfg.sl.RSPort);
		IPCLogP(eLog.LogApp,CRITICAL,FILERROR,eLog.LogMsg);
		return(result);
	}

#if SLDEBUG
	printf("- ThSelfLevl. I!. SetSLPort(): completed [%d].\n",result);
#endif
	return(result);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
bool SLDataSend()
{
	bool SendError = FALSE;
	int S,N = 3;

	S = write(hRSPort,SndSLBuffer,N);
	if(S != N) {
#if SLDEBUG
		printf("- ThSelfLevl. E!. Error %d in write\n",errno);
#endif
		SendError = TRUE;
	}
#if SLDEBUG
	else {
		printf("- ThSelfLevl. E!. SLDataSend():");
		for(int i=0;i<N;i++)
			printf(" %02X",SndSLBuffer[i]);
		printf("\n");
	}
#endif

	return(SendError);
}

/* -------------------------------------------------------------------------------------------------------
   Discharge old data from UART memory buffer
------------------------------------------------------------------------------------------------------- */
void SLClearBuffer(void)
{
	char RSByte;
	while(read(hRSPort,&RSByte,1) > 0) {};
}

/* -------------------------------------------------------------------------------------------------------
   Read a SL stream from RS port
   return:
   n > 0 : number of character read
   0     : some timeout (no data received) when 100 missing data
------------------------------------------------------------------------------------------------------- */
int SLDataRecv()
{
	int  i,N=0,TOut=0,
		 DataLen = 0;
	char RSByte,LRC;
	char buff[MAXSLMSGLEN] = { 0 };

	// ------------------------------------------------------
	// Ignore all characters until a known cmd
	// Timeout break after 100 null reads: return 0
	// ------------------------------------------------------
	do {
		// se legge un carattere lo analizza
		if(read(hRSPort,&RSByte,1) > 0) {
			// finchè non trova l'inizio sequenza
			if(RSByte == SLSTAT ||
			   RSByte == SLREPORT ||
			   RSByte == SLHEALTH) {
				// Questo è il byte buono della sequenza: esci
				buff[N=0] = RSByte;
				TOut = 0;
				break;
			}
			// Altrimenti incrementa il contatore di timeout
			else
				++TOut;
		}
		// Altrimenti incrementa il contatore di timeout
		else
			++TOut;

	}while(TOut <= RSCHARTOUT);

	// ------------------------------------------------------
	// Timeout on msg identifier
	// ------------------------------------------------------
	if(TOut > RSCHARTOUT) return(0);

	// ------------------------------------------------------
	// Expected data bytes including LRC
	// ------------------------------------------------------
	if(buff[0] == SLSTAT) DataLen = 2;
	else if(buff[0] == SLREPORT) DataLen = 7;
	else if(buff[0] == SLHEALTH) DataLen = 7;

	// ------------------------------------------------------
	// Read all data character character
	// ------------------------------------------------------
	do {
		if(read(hRSPort,&RSByte,1) > 0) {
			buff[++N] = RSByte;
			TOut = 0;
		}
		else if(++TOut > RSCHARTOUT) break;
	} while(N < DataLen);

	// ------------------------------------------------------
	// Timeout on data
	// ------------------------------------------------------
	if(TOut > RSCHARTOUT) return(0);

	// ------------------------------------------------------
	// Message complete
	// ------------------------------------------------------
	buff[++N] = '\0';

#if SLDEBUG
	printf("- ThSelfLevl. I!. SLDataRecv(): ");
	for(i=0;i<N;i++)
		printf(" %02X",buff[i]);
	printf("\n");
#endif

	// ------------------------------------------------------
	// Check the LRC
	// ------------------------------------------------------
	LRC = 0x00;
	for(i=0;i<=DataLen;i++)
		LRC ^= buff[i];
	if(LRC != buff[i]) {
		sprintf(eLog.LogMsg,"LRC error on Self-Level RS interface");
		IPCLogP(eLog.LogApp,CRITICAL,PROTERR,eLog.LogMsg);
#if SLDEBUG
		printf("- ThSelfLevl. W!. LRC error\n");
#endif
		return(0);						// CRC error
	}

	// ------------------------------------------------------
	// Regular exit
	// ------------------------------------------------------
	memcpy(RcvSLBuffer,buff,N);
	return(N);
}


