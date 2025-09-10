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

#include <unistd.h>

#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>

//#include <sys/ioctl.h>
#include <sys/ioctl.h>

//#include <linux/can/raw.h>

#include "main.h"

//These declarations come from the various include files: sys/ioctl.h, net/if.h & linux/can.h

//Check that you have these files in /usr/include/

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

	// -----------------------------------------------------------------------
	// Next, we must retrieve the interface index for the interface name
	// (can0, can1, vcan0 etc) we wish to use. To do this we send an I/O
	// control call and pass an ifreq structure containing the interface name:
	// -----------------------------------------------------------------------
	struct ifreq ifr;
	//strcpy(ifr.ifr_name, "vcan0" );
	strcpy(ifr.ifr_name, "can0" );
	ioctl(s,SIOCGIFINDEX,&ifr);

	// -----------------------------------------------------------------------
	// Alternatively, if you use zero as the interface index, you can retrieve
	// packets from all CAN interfaces.
	// Armed with the interface index, we can now bind the socket to the CAN Interface:
	// -----------------------------------------------------------------------
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	   perror("Bind");
	   return 1;
	}

	// -----------------------------------------------------------------------
	// Below we initialise a CAN frame with an ID of 0x555, a payload of 5 bytes
	// containing “hello” and send it using the write() system call:
	// -----------------------------------------------------------------------
	struct can_frame frame;
	frame.can_id = 0x555;
	frame.can_dlc = 5;
	memcpy(&frame.data,"Hello",5);  // <<<<<<<<<<<<<<  ATTENZIONE !
	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	   perror("Write");
	   return 1;
	}

	exit(0);
}


