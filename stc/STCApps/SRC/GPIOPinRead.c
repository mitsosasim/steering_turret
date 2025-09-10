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
  int RC;

  if(argc < 2) 
  {
    fprintf(stderr, "\nUsage:\t%s { Pin } \n"
      "\tPin    : Pin in the GPIO connector on ROCKPro64 board.\n",
      argv[0]);
    exit(1);
	}
	PinNo = strtoul(argv[1], 0, 0);
  
  RC= GPIOInit();
  if( RC != 0) return RC;
  RC= GPIOPinInit( PinNo, GPIOPinModeGPIO);
  if( RC != 0) return RC;
  RC= GPIOSetPinDirection( PinNo , GPIOPinDirectionInput );
  if( RC != 0) return RC;
  RC= GPIOGetPinState( PinNo );
  fprintf( stdout, "Pin %d: %d\n", PinNo, RC);
  
  GPIOPinRelease( PinNo);

  return 0;
}

