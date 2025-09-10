/*
 *
 */
#include "ROCKPro64GPIO.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdint.h>   /* for uint64_t */

int main(int argc, char **argv) 
{
  int PinNo;
  int Val;
  int RC;

  if(argc < 3) 
  {
    fprintf(stderr, "\nUsage:\t%s { Pin } { 0/1 }\n"
      "\tPin    : Pin in the GPIO connector on ROCKPro64 board.\n"
       "\t0/1    : Second argument must 0 or 1.\n",
     argv[0]);
    exit(1);
	}
	PinNo = strtoul(argv[1], 0, 0);
	Val = strtoul(argv[2], 0, 0);
  if( Val != 0 && Val != 1)
  {
    fprintf(stderr, "\nUsage:\t%s { Pin } { 0/1 }\n"
      "\tPin    : Pin in the GPIO connector on ROCKPro64 board.\n"
      "\t0/1    : Second argument must 0 or 1.\n",
      argv[0]);
    exit(1);
	}
  RC= GPIOInit();
  if( RC != 0) return RC;
  RC= GPIOPinInit( PinNo, GPIOPinModeGPIO);
  if( RC != 0) return RC;
  RC= GPIOSetPinDirection( PinNo , GPIOPinDirectionOutput );
  if( RC != 0) return RC;
  RC= GPIOSetPinState( PinNo, (enum GPIOPinState) Val );
  if( RC != 0) fprintf(stderr, "Function GPIOSetPinState() returned non-zero status\n");
  GPIOPinRelease( PinNo);

  return 0;
}

