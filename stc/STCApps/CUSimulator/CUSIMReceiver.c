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
#include <string.h>
#include <pthread.h>

//#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//#include <netinet/in.h>

#include "CUSim.h"

extern int    sockfd;
extern struct sockaddr_in servaddr;
extern pthread_mutex_t lock;

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void* ThSIMReceiver(void *arg)
{
    int  n;
    char buffer[MAXLINE];
    char ServerName[16];
    socklen_t len;

    len = sizeof(servaddr);
	printf("CU simulator: receiver is ON\n");
    do {
    	memset(buffer,0,sizeof(buffer));

        n = recvfrom(sockfd,(char *)buffer,MAXLINE,MSG_WAITALL,
                    (struct sockaddr *)&servaddr,&len);
        buffer[n] = '\0';

    	// -----------------------------------------------------------
		strcpy(ServerName,inet_ntop(AF_INET,&servaddr.sin_addr,
				ServerName,sizeof(ServerName)));
        printf("Simulator [%s] Received: %s\n",ServerName,buffer);
	}while(1);

}


