/* =======================================================================================================
  CUSim.c
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "STCTypes.h"
#include "STCProtocol.h"
#include "CUSim.h"

#define PORT    9966

int    sockfd;
struct sockaddr_in servaddr;
pthread_mutex_t lock;

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	// -------------------------------------------------------------------
    memset(&servaddr,0,sizeof(servaddr));

	// -------------------------------------------------------------------
    // Creating socket file descriptor
	// -------------------------------------------------------------------
    if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) < 0 ) {
        printf("CU simulator E!. Socket creation failed\nExit");
        exit(1);
    }

	// -------------------------------------------------------------------
    // Filling server information
	// -------------------------------------------------------------------
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

	// -----------------------------------------------------------
	// Initialize mutex
	// -----------------------------------------------------------
    if(pthread_mutex_init(&lock, NULL) != 0) {
        printf("Mutex init has failed\nExit");
        exit(1);
    }

	// -----------------------------------------------------------
	// -----------------------------------------------------------
    //printf("CU simulator on local port %d\n",PORT);

	// -----------------------------------------------------------
	// Create threads
	// -----------------------------------------------------------
    pthread_t thr[2];
    pthread_create(&(thr[0]),NULL,&ThSIMSender,NULL);
    pthread_create(&(thr[0]),NULL,&ThSIMReceiver,NULL);

	while(1)
		sleep(99);

    close(sockfd);
}

/* -------------------------------------------------------------------------------------------------------
   Check UDP connection
------------------------------------------------------------------------------------------------------- */
void SimCheckUDP()
{
	int 	n;
    char 	header[32];
	char    RcvBuffer[MAXJSONLEN],
			SndBuffer[MAXJSONLEN];
    socklen_t len;

    len = sizeof(servaddr);


	// -----------------------------------------------------------
    JSONheader(header);
    sprintf(SndBuffer,"{ %s }",header);

	// -----------------------------------------------------------
    pthread_mutex_lock(&lock);
   	sendto(sockfd,(const char *)SndBuffer,strlen(SndBuffer),
   			MSG_DONTWAIT,(const struct sockaddr *)&servaddr,sizeof(servaddr));
    pthread_mutex_unlock(&lock);
    printf("Simulator Sent: %s\n",SndBuffer);

	// -----------------------------------------------------------
    pthread_mutex_lock(&lock);
    n = recvfrom(sockfd,(char *)RcvBuffer,MAXJSONLEN,MSG_WAITALL,
				(struct sockaddr *)&servaddr,&len);
	RcvBuffer[n] = '\0';
    pthread_mutex_unlock(&lock);
    printf("Simulator Received: %s\n",RcvBuffer);

    // check interface matches (known AppID and matching protocol version)
 }


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void SimAskBIT(){}								// Ask the Turret to perform a complete BIT
void SimAskStatus(){}							// Request the Turret full current status
void SimSetStart(){}								// Start Turret activities
void SimSetStop(){}								// Set the Turret in safe mode and stop all activities
void SimSetToZero(){}							// Move all motors to their zero angle (initial position)
void StartLevelling(){}							// Start Self-level

// Turret commands
void SimMoveTurretTo(float AbsAngle){}			// move the Turret to absolute position
void SimMoveTurretBy(float RelAngle){}			// move the Turret by a relative angle +/- alfa

// Mirrors commands
void SimMoveMirrorsTo(float AbsAngle){}			// move all mirrors to absolute position
void SimMoveMirrorsBy(float RelAngle){}			// move all mirrors by a relative angle +/- alfa
void SimMoveMirrorTo(int id,float AbsAngle){}	// move mirror id to absolute position
void SimMoveMirrorBy(int id,float RelAngle){}		// move mirror by a relative angle +/- alfa

// SUTs commands
void SimMoveSUTsTo(float AbsAngle){}			// move all SUTs to absolute position
void SimMoveSUTsBy(int id,float RelAngle){}		// move all SUTs by a relative angle +/- alfa
void SimMoveSUTTo(int id,float AbsAngle){}		// move SUT id to absolute position
void SimMoveSUTBy(int id,float RelAngle){}		// move SUT id by a relative angle +/- alfa

// Laser commands
void SimLasers(int light){}						// Switch all laser 1=on/0=off
void SimLaser(int id,int light){}				// Switch laser id 1=on/0=off

// Camera commands
void MoveCameraBy(float RelAngle){}				// Move camera motor by a relative angle +/- alfa
