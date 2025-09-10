/* =======================================================================================================
  ThBuzzer.c
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
#include <gpiod.h>

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
#include "STCShData.h"

#define GPIO_CHIP 	"/dev/gpiochip1"	// Might be gpiochip 0/1 depending on the board
#define GPIO_NAME 	"GPIO1_A1"

struct  gpiod_chip *chip;
struct  gpiod_line *BuzzerIOLine;
static  UINT bank = 0,
			 offset = 0,
			 number = 0;

int  GetGPIOata(char *);
void SetBuzzer(bool);
void SetLamp(bool);


static LOGMSG   eLog;

/* -------------------------------------------------------------------------------------------------------
   This thread manages the Buzzer, the safety lamp and teh radar
------------------------------------------------------------------------------------------------------- */
void* ThSafety(void* thr_data)
{
    int      ret;
    int 	 WaitTime,DownTime;
	int		 cnt=0,NSecs=0;
    unsigned int LineId;
    STCSTATE STstate;

    // -------------------------------------------------------------------
    memset(&eLog,0,sizeof(eLog));
	eLog.LogApp = STCCONTROLLER;
	sprintf(eLog.LogMsg,"ThSafety thread started");
	IPCLogP(eLog.LogApp,INFOLOG,0,eLog.LogMsg);

	// -------------------------------------------------------------------
	printf("- ThSafety.I!.\n");

    /* --------------------------------------------------------------------------------------
	   Open the GPIO chip
	-------------------------------------------------------------------------------------- */
	pthread_mutex_lock(&DIOlock);
    LineId = GetGPIOata(GPIO_NAME);
    //printf("Open chip:\n");
    chip = gpiod_chip_open(GPIO_CHIP);
    if(!chip) {
    	sprintf(eLog.LogMsg,"ThSafety: GPIO chip access error");
    	IPCLogP(eLog.LogApp,CRITICAL,0,eLog.LogMsg);
    	printf("%s",eLog.LogMsg);
    	pthread_exit((void*)0);;
    }
    // Get the GPIO line
    //printf("Get line:\n");
    BuzzerIOLine = gpiod_chip_get_line(chip,LineId);
    if(!BuzzerIOLine) {
        gpiod_chip_close(chip);
    	sprintf(eLog.LogMsg,"ThSafety: GPIO line error");
    	IPCLogP(eLog.LogApp,CRITICAL,0,eLog.LogMsg);
    	printf("%s",eLog.LogMsg);
    	pthread_exit((void*)0);;
    }
    // Request the line as output, initial value LOW
    //printf("Request as output:\n");
    ret = gpiod_line_request_output(BuzzerIOLine,"gpio_test", 0);
    if (ret < 0) {
        gpiod_chip_close(chip);
    	sprintf(eLog.LogMsg,"ThSafety: GPIO line request error");
    	IPCLogP(eLog.LogApp,CRITICAL,0,eLog.LogMsg);
    	printf("%s",eLog.LogMsg);
    	pthread_exit((void*)0);;
    }
    // Set GPIO LOW
    //printf("GPIO set LOW\n");
    gpiod_line_set_value(BuzzerIOLine,0);
    pthread_mutex_unlock(&DIOlock);


	// -------------------------------------------------------------------
    WaitTime = (STCCfg.safety.MoveDelay * 10);
	DownTime = WaitTime;

	// -------------------------------------------------------------------
	// Infinite loop checking the state of the turret related with Buzzer
	// -------------------------------------------------------------------
	while(TRUE) {
		usleep(100000);  // 100000 microseconds = 100 milliseconds

		// The CU is not connected
		if(!SysGetCUConnected()) {
			SetBuzzer(FALSE);
			SetLamp(FALSE);
			DownTime = WaitTime;
			NSecs = STCCfg.safety.MoveDelay;
			if(SysMovingEnabled())
				SysEnableMoving(FALSE);
			continue;
		}

		// DownTime before enabling the ST moving
		if(DownTime > 0) {
			if(++cnt == 10) {
				cnt = 0;
				NSecs--;
				printf("- ThSafety. %d\n",NSecs);
			}
			if(--DownTime == 0) {
				SysEnableMoving(TRUE);
				printf("- ThSafety. Movement enabled\n");
			}
		}

		STstate=SysGetState();

		// When the buzzer is enabled, manages the buzzer
		if(STCCfg.safety.Buzzer) {
			if(STstate == OPER)
				SetBuzzer(TRUE);
			else
				SetBuzzer(FALSE);
		}
		// When lamp is enabled, manages the lamp
		if(SysMovingEnabled())
			SetLamp(TRUE);
		else
			SetLamp(FALSE);

	}
	return(0);
}

/* --------------------------------------------------------------------------------------
	The formula for converting GPIO<bank>_<letter><number> to a GPIO number is:
	GPIO number = [(bank × 32) + (letter_offset × 8) + number]%32  (modulo 32)
	Letter offset: A = 0, B = 1, C = 2, D = 3
	GPIO1_C2 = (1 × 32) + (2 × 8) + 2 = 32 + 16 + 2 = 50
-------------------------------------------------------------------------------------- */
int GetGPIOata(char *p)
{
	int i=0,id,Line;
	char ch;

	while(p[i] != '\0') {
		ch = p[i];
		switch(i) {
		case 4:
			bank = ch - 48;
			break;
		case 6:
			offset = ch - 65;
			break;
		case 7:
			number = ch -48;
			break;
		}
		++i;
	}
	id = (bank * 32) + (offset * 8) + number;
	Line = id%32;
    printf("\t\t%s => pin: %d => line: %d\n",GPIO_NAME,id,Line);
    printf("\t\t    ^ ^^\n");
    printf("\t\t    | |+- number: %d\n",number);
    printf("\t\t    | +-- offset: %d\n",offset);
    printf("\t\t    +---- bank:   %d\n",bank);
    return(Line);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SetBuzzer(bool BuzzerON)
{
	static bool BuzzState = FALSE;

	pthread_mutex_lock(&DIOlock);
	if(BuzzerON) {
		if(!BuzzState) {
			printf("ThSafety. Beep ON\n");
			gpiod_line_set_value(BuzzerIOLine,1);
			BuzzState = BuzzerON;
		}
	}
	else {
		if(BuzzState) {
			printf("ThSafety. Beep OFF\n");
			gpiod_line_set_value(BuzzerIOLine,0);
			BuzzState = BuzzerON;
		}
	}
    pthread_mutex_unlock(&DIOlock);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SetLamp(bool LampON)
{
	static bool LampState = FALSE;

	pthread_mutex_lock(&DIOlock);
	if(LampON) {
		if(!LampState) {
			printf("- ThSafety. Lamp ON\n");
		//	gpiod_line_set_value(BuzzerIOLine,1);
			LampState = LampON;
		}
	}
	else {
		if(LampState) {
			printf("- ThSafety. Lamp OFF\n");
		//	gpiod_line_set_value(BuzzerIOLine,0);
			LampState = LampON;
		}
	}
    pthread_mutex_unlock(&DIOlock);
}

