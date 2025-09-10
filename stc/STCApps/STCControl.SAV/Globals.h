#ifndef _GLOBALS
#define _GLOBALS
#ifdef INCLUDE_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

#include "STCTypes.h"
#include "STCShData.h"

EXTERN STCSYS STCSys;					// System Data
EXTERN STCCFG STCCfg;					// Config Data
EXTERN UDPCOM udp;						// All UDP data
EXTERN SMGNSS gps;						// GPS data

EXTERN pthread_mutex_t Netlock;			// NetLock;
EXTERN pthread_mutex_t Datalock;		// DataLock;
EXTERN pthread_mutex_t CANlock;			// CANLock;
EXTERN pthread_mutex_t DIOlock;			// DGIO Lock;

EXTERN char EventList[16];
EXTERN int  EventPtr;
EXTERN int  TimeoutCounter;				// Check CU timeout

#endif
