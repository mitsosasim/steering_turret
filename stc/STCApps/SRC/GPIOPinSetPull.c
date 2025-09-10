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

  if(argc < 3) 
  {
    fprintf(stderr, "\nUsage:\t%s { Pin } { pull }\n"
      "\tPin    : Pin in the GPIO connector on ROCKPro64 board.\n"
      "\tpull   : up/down/normal inital letters are enough.\n",
      argv[0]);
    exit(1);
	}
	PinNo = strtoul(argv[1], 0, 0);
  
  RC= GPIOInit();
  if( RC != 0) return RC;
  RC= GPIOPinInit( PinNo, GPIOPinModeGPIO);
  if( RC != 0) return RC;
  if( argv[2][0] == 'u' )
  {
    RC= GPIOSetPullUpDown( PinNo, GPIOPullUpDownStateUp );
    return RC;
  } else if( argv[2][0] == 'd' )
  {
      RC= GPIOSetPullUpDown( PinNo, GPIOPullUpDownStateDown );
      return RC;
  } else if( argv[2][0] == 'n' )
  {
    RC= GPIOSetPullUpDown( PinNo, GPIOPullUpDownStateNormal);
    return RC;
  }
  
  
  GPIOPinRelease( PinNo);

  return 0;
}

