/*===========================================================================
 Name        : ThCANOpen.c
 Author      : R. Filippone
 Version     :
 Copyright   : ALMA sistemi srl
 Description :
 ---------------------------------------
 Error codes:
	CUCom: error 200 - 
 ==========================================================================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <pthread.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"
#include "Globals.h"

#include "STCControl.h"
#include "STCShData.h"

static LOGMSG   eLog;

void InitCANData();

/*---------------------------------------------------------------------
---------------------------------------------------------------------*/
void* ThCANOperator(void* thr_data)
{
	// -------------------------------------------------------------------
    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCCONTROLLER;
	sprintf(eLog.LogMsg,"CANopen management thread started");
	IPCLogErrorP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);

	InitCANData();

	// -------------------------------------------------------------------
	printf("- ThCANOper .I!. started\n");

	// -------------------------------------------------------------------
	while(true)
		sleep(88);

	return(0);
}

/*---------------------------------------------------------------------
---------------------------------------------------------------------*/
void InitCANData()
{
	pthread_mutex_lock(&Netlock);
	// ---------------------------------------------------------------
	// Turret Motor
	// ---------------------------------------------------------------
	// ---------------------------------------------------------------
	// Camera Motor
	// ---------------------------------------------------------------
	// ---------------------------------------------------------------
	// SUT Motors
	// ---------------------------------------------------------------
	// ---------------------------------------------------------------
	// Mirror Motors
	// ---------------------------------------------------------------
	// ---------------------------------------------------------------
	// Environmental
	// ---------------------------------------------------------------
	// ---------------------------------------------------------------
	// Position
	// ---------------------------------------------------------------
	gps.latitude  = 41.889896f;
	gps.longitude = 12.491316f;
	gps.Altitude  = 27.0f;
	gps.Heading   =  0.12345678f;
	gps.Pitch	  =  0.01f;
	gps.Roll	  =  0.01f;
	memset(gps.UTCtime,0,10);
    pthread_mutex_unlock(&Netlock);
}
