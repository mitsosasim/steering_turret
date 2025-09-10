/*
 * main.c
 *
 *  Created on: Feb 3, 2025
 *      Author: alma
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>	// queue
#include <sys/shm.h>	// shared memory
#include <sys/sem.h>	// semaphore

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"

#include "main.h"

// IPC
static int shmid;		// shared memory
static int semid;		// semaphore
static int queid;

static struct sembuf shlock   = {0,-1, 0},
			  	     shunlock = {0, 1, 0};
static SMGNSS SHLocData,*pSHData;

void ShowData(void);
void TestFile();

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	memset(&SHLocData,0,sizeof(SMGNSS));
	pSHData = &SHLocData;


	LOGMSG TMsg;

	//TestFile();

	// -------------------------------------------------------------------
	// I P C - Shared memory
	// -------------------------------------------------------------------
	struct shmid_ds *iMem;
	int R = 0;

	// create the Shared memory
    shmid = shmget(GPSDATASHMKEY,sizeof(SMGNSS),IPC_CREAT | IPC_EXCL | O_RDWR);
    if(shmid == -1 &&  errno != EEXIST) { 					// SM already exist
    	printf("SM access error\nExit\n");
    	if(errno == EACCES) R = 1;
    	if(errno == EEXIST) R = 2;
		if(errno == ENOENT) R = 3;
		if(errno == ENOMEM) R = 4;
		if(errno == ENOSPC) R = 5;
        exit(1);
    }
    //ftruncate(shmid,sizeof(SMGNSS));


	// to control the Shared memory
    shmctl(shmid,IPC_STAT,iMem);

	// map the Shared memory to local structure
	pSHData = shmat(shmid,NULL,0);
	if(pSHData == NULL) {
    	printf("SM attach error\nExit\n");
        exit(1);
    }

	// -------------------------------------------------------------------
	// I P C - Semaphore
	// -------------------------------------------------------------------
 	//semid = semget(GPSDATASEMKEY,1,O_CREAT | IPC_EXCL | O_RDWR);
 	semid = semget(GPSDATASEMKEY,1,IPC_CREAT | IPC_EXCL | O_RDWR);
	if(semid == -1 && errno == EEXIST) { 					// Sem already exist
    	printf("SEM access error\nExit\n");
        exit(1);
    }
	semop(semid,&shunlock,1);

	// -------------------------------------------------------------------
	// I P C - Queue
	//  /proc/sys/kernel/msgmnb defines the max size of the queue; then
	//  the queue can acceptt N/sizeof(LOGMSG) messages.
	// -------------------------------------------------------------------
	//int R;
	struct msqid_ds iQue;

	queid = msgget(LOGDATAQUEKEY,IPC_CREAT | 0600);
	if(queid == -1 && errno == EEXIST) { 					// Queue already exist
    	printf("QUE access error\nExit\n");
        exit(1);
    }
	msgctl(queid,IPC_STAT,&iQue);
	printf("Q size = %ld (= %ld messages)\n",iQue.msg_qbytes,(iQue.msg_qbytes/sizeof(LOGMSG)));

	//msgrcv(queid,&logmsg,sizeof(LOGMSG),0,0);
	strcpy(TMsg.LogTime,"hhmmss,mmm");
	TMsg.LogLev = CRITICAL;
	TMsg.LogErrCode = 1001;
	TMsg.LogApp = 9;
	sprintf(TMsg.LogMsg,"Test Coda degli errori");


	// -------------------------------------------------------------------
	// Test the Queue
	// -------------------------------------------------------------------
	R=msgsnd(queid, &TMsg, sizeof(LOGMSG), IPC_NOWAIT);
	if(R < 0) {
    	printf("QUE send error\nExit\n");
		exit(1);
	}

	// -------------------------------------------------------------------
	// Fill the Queue
	/* -------------------------------------------------------------------
	do {
		TMsg.LogErrCode++;
		R=msgsnd(queid, &TMsg, sizeof(LOGMSG), IPC_NOWAIT);
		if(R < 0) {
			printf("QUE send error\nExit\n");
			exit(1);
		}
		sleep(2000);
	}while(1); */


	// -------------------------------------------------------------------
	printf("\033[2J"); // Clear screen
	do {
		sleep(1);
		if(!semop(semid,&shlock,1)) {
		    memcpy(&SHLocData,pSHData,sizeof(SMGNSS));
			semop(semid,&shunlock,1);
			ShowData();
		}

	}while(1);

	return(0);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void ShowData()
{
	char SolStat[32];
	static unsigned int i=0;


	if(i > 0xFFFF)
		i = 0;
	printf("\033[1;1H \n%d\n",i++);

	// ------------------------------------------------------
	//  '/v' was log of GPS data was requested
	// ------------------------------------------------------
	if(SHLocData.SolStat == SOL_COMPUTED)					// Solution computed
		strcpy(SolStat,"Computed!");
	else if(SHLocData.SolStat == INSUFFICIENT_OBS)		// Insufficient observation
		strcpy(SolStat,"Insufficient obs.");
	else if(SHLocData.SolStat == NO_CONVERGENCE)			// No convergence, invalid solution
		strcpy(SolStat,"No convergence");
	else if(SHLocData.SolStat == COV_TRACE)				// Covariance matrix trace exceeds maximum (trace > 1000 m)
		strcpy(SolStat,"Covariance error");
	if(SHLocData.HeadStat == INVALID)
		strcat(SolStat,"/Invalid");
	if(SHLocData.HeadStat == FIXED)
		strcat(SolStat,"/Fixed");
	if(SHLocData.HeadStat == FLOAT)
		strcat(SolStat,"/Float");

	printf("- Date:Time: %s\n",SHLocData.UTCtime);
	printf("  - Solution:    %s\n",SolStat);
	printf("  - Latitude:    %10.6lf %c\t\t",SHLocData.latitude,SHLocData.latd);
	printf("  - Altitude:    %10.6lf\n",SHLocData.Altitude);
	printf("  - Longitude:   %10.6lf %c\t\t",SHLocData.longitude,SHLocData.lond);
	printf("  - Pitch:       %10.6lf\n",SHLocData.Pitch);
	printf("  - Heading:     %10.6lf\t\t",SHLocData.Heading);
	printf("  - Roll:        %10.6lf\n",SHLocData.Roll);
	printf("  - Speed OG:    %10.6lf\n",SHLocData.Speed);
	printf("\n");
}


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void TestFile()
{
	int  len  = 255;
	int  read = 0;
	char NMEAbuffer[255];
	char SrcName[256];
	FILE *hSrcf;

	strcpy(SrcName,"/home/alma/STCApps/data/capture1");

	if((hSrcf = fopen(SrcName,"r")) == NULL)
		return;
	else {
		while(1) {
			if(fgets(NMEAbuffer, sizeof(NMEAbuffer), hSrcf) == NULL) {
			//read = getline(NMEAbuffer,&len,hSrcf);
			//if(feof(hSrcf)) {
				fseek(hSrcf,0,SEEK_SET);
				printf("\n\n  File reload ... \n\n");

			//	fclose(hSrcf);
			//	hSrcf = NULL;
			//	read = 0;
			//	printf("\n\nSource file completed.\n\n");
			}
			printf("[%d=>%s]\n",read,NMEAbuffer);
		}
		fclose(hSrcf);
	}
}
