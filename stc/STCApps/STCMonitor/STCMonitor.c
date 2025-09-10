/* =======================================================================================================
  main.c
  Created on: Feb 13, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine

 ------------------------------------------------------------------------------
   v. 1.0 - Feb 13, 2025 - Monitor the STC controller
 ------------------------------------------------------------------------------
  Reference:
   R1 - Reference (document, web site, other source files, etc.)
   R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>	// queue
#include <sys/shm.h>	// shared memory
#include <sys/sem.h>	// semaphore

#include <pthread.h>
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <sys/stat.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"

#include "BCounter.h"
#include "STCShData.h"
#include "STCMonitor.h"


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    pthread_t thrid[10];
    int i;

	// -------------------------------------------------------------------
	// I P C - Initialization
	// -------------------------------------------------------------------
	if(IPCInitialize() < 0) {
    	printf("STCMonitor E!. IPC initialization error\nExit\n");
        exit(1);
	}

	pthread_create(&thrid[i=0],NULL,CheckGPSData,NULL);
	pthread_create(&thrid[++i],NULL,CheckResources,NULL);


	while(true)
		sleep(1000);

}


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void* CheckGPSData(void* p)
{
	static SMGNSS GPSData;

	// -------------------------------------------------------------------
	printf("\033[2J"); // Clear screen

	printf("STCMonitor I!. start monitoring\n");
	do {
		IPCGetGPSData(&GPSData);
		ShowGPSData(&GPSData);
	    sleep(1);
	}while(true);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void* CheckResources(void* p)
{
	int C=0,
	    myid = getpid();
	do {
	    sleep(1);
	   // printf("CheckResources. Thread %d - Counter: %d\n",myid,++C);
	}while(true);

}


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void ShowGPSData(SMGNSS *ShGPSData)
{
	char SolStat[32];
	static unsigned int i=0;

	if(i > 0xFFFE)
		i = 0;
	printf("\033[1;1H \n%d\n",i++);

	// ------------------------------------------------------
	//  '/v' was log of GPS data was requested
	// ------------------------------------------------------
	if(ShGPSData->SolStat == SOL_COMPUTED)					// Solution computed
		strcpy(SolStat,"Computed!");
	else if(ShGPSData->SolStat == INSUFFICIENT_OBS)		// Insufficient observation
		strcpy(SolStat,"Insufficient obs.");
	else if(ShGPSData->SolStat == NO_CONVERGENCE)			// No convergence, invalid solution
		strcpy(SolStat,"No convergence");
	else if(ShGPSData->SolStat == COV_TRACE)				// Covariance matrix trace exceeds maximum (trace > 1000 m)
		strcpy(SolStat,"Covariance error");
	if(ShGPSData->HeadStat == INVALID)
		strcat(SolStat,"/Invalid");
	if(ShGPSData->HeadStat == FIXED)
		strcat(SolStat,"/Fixed");
	if(ShGPSData->HeadStat == FLOAT)
		strcat(SolStat,"/Float");

	printf("- Date:Time: %s\n",ShGPSData->UTCtime);
	printf("  - Solution:    %s\n",SolStat);
	printf("  - Latitude:    %10.6lf %c\t\t",ShGPSData->latitude,ShGPSData->latd);
	printf("  - Altitude:    %10.6lf\n",ShGPSData->Altitude);
	printf("  - Longitude:   %10.6lf %c\t\t",ShGPSData->longitude,ShGPSData->lond);
	printf("  - Pitch:       %10.6lf\n",ShGPSData->Pitch);
	printf("  - Heading:     %10.6lf\t\t",ShGPSData->Heading);
	printf("  - Roll:        %10.6lf\n",ShGPSData->Roll);
	printf("  - Speed OG:    %10.6lf\n",ShGPSData->Speed);
	printf("\n");
}

