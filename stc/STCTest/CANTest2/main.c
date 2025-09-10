/* =======================================================================================================
  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include <unistd.h>

//#include <net/if.h>
//#include <sys/ioctl.h>
#include <sys/socket.h>

//#include <linux/can.h>
//#include <linux/can/raw.h>

#include "main.h"

/* -------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	// -----------------------------------------------------------------------
	// Opening and binding to a CAN socket
	// The first step before doing anything is to create a socket.
	//   This function accepts three parameters – domain/protocol family
	//   (PF_CAN), type of socket (raw or datagram) and socket protocol.
	//   If successful, the function then returns a file descriptor.
	// -----------------------------------------------------------------------
	int s;
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
	   perror("Socket");
	   return 1;
	}
	exit(0);
}


