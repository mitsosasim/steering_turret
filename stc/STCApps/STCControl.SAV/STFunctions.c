/* =======================================================================================================
  STFunctions.c
  Created on: Jul 14, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Jul 14, 2025 - Short description
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
#include <errno.h>
#include <unistd.h>

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"
#include "Globals.h"

#include "STCControl.h"
#include "STCShData.h"


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STReboot()
{
	int CountDown = 3;

	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"Reboot request");
	printf("- STFunction. I!. Reboot() in %d seconds.\n",CountDown);

	do {
		sleep(1);
		printf("%d\n",--CountDown);
	}while(CountDown);

	system("sudo /usr/bin/systemctl reboot");
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STShutDown()
{
	int CountDown = 5;

	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"Graceful shutdown request");
	printf("- STFunction. I!. Shutdown() in %d seconds.\n",CountDown);

	do {
		sleep(1);
		printf("%d\n",--CountDown);
	}while(CountDown);

	system("sudo /usr/bin/systemctl poweroff");
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STDeleteLog(int p)
{
	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"STDeleteLog");
	printf("- STFunction. I!. STDeleteLog().\n");
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STStoragePosition()
{
	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"STStoragePosition");
	printf("- STFunction. I!. STStoragePosition().\n");
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STResetAll()
{
	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"STResetAll");
	printf("- STFunction. I!. STResetAll().\n");
	STCCfg.cu.UpdTime = 300;
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STSelfTest()
{
	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"STSelfTest");
	printf("- STFunction. I!. STSelfTest().\n");
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void STRptSWVersion()
{
	IPCLogErrorP(STCCONTROLLER,INFOLOG,PROTERR,"STRptSWVersion");
	printf("- STFunction. I!. STRptSWVersion().\n");
}

