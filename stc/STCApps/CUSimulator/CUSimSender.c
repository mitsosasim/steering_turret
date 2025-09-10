/* =======================================================================================================
  CUSIMReceiver.c
  Created on: Feb 26, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 26, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <string.h>
#include <pthread.h>

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <arpa/inet.h>
//#include <netinet/in.h>

#include "STCTypes.h"
#include "STCProtocol.h"

#include "CUSim.h"

extern int    sockfd;
extern struct sockaddr_in servaddr;
extern pthread_mutex_t lock;


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void* ThSIMSender(void *arg)
{
    int  n;
    char buffer[MAXLINE];

    JSONinit(STCSIMULATOR);

    printf("CU simulator: sender is ON\n");
    do {
        SimCheckUDP();
		sleep(10);	// seconds
	}while(1);

}
