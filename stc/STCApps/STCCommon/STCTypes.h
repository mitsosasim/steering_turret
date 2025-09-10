/* =======================================================================================================
  STCTypes.h

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
 ------------------------------------------------------------------------------


======================================================================================================= */
#ifndef TYPES_H_
#define TYPES_H_

#include <stdbool.h>

#define UINT	unsigned int
#define ULONG	unsigned long int
#define BYTE	unsigned char

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
typedef struct _gnsst {
	unsigned int year;
	unsigned int month;
	unsigned int day;
	unsigned int hour;
	unsigned int minute;
	unsigned int second;
	char  		 GDO[20];
}GNSS_T;


typedef enum _state {
	INIT = 1,				// At startup, until the first Self-Test is complete
	OPER,					// The Turret is always Operative
	DOWN,					// The ST is shutting down (in 5 seconds)
}STCSTATE;
typedef enum _mode {
	UNDEF = 0,				// When not applicable
	FULL = 1,				// Everything is OK. Waiting for CU cmds to be properly executed
	FAIL,					// A critical failure was detected and the ST cannot work properly or safely
	SELFT,					// The Turret keeps cmd in queue until Self-Test is not complete.
	BUSY,					// The Turret keeps cmd in queue until current operation is not complete.
}STCMODE;

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
typedef union _semun {
	int val;				// value for SETVAL
	struct semid_ds *buf;	// buffer for IPC_STAT, IPC_SET
	unsigned short *array;	// array for GETALL, SETALL
						   	// Linux specific part:
	struct seminfo *__buf;	// buffer for IPC_INFO
}SEMUN;

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
// Log level
typedef enum _loglev {
	DISABLED = 0,			// Disabled
	CRITICAL,				// Critical Error
	ERRLOG,					// Unblocking Error
	WARNLOG,				// Warning
	INFOLOG,				// Informations
}LOGLEV;

// Known application identifiers
typedef enum _appid {		// App IDs. DO NOT CHANGE !
	STCCUNIT	  = 11,		// Control Unit app
	STCGPSDATA	  = 12,		// GPS reader
	STCLOGGER	  = 13,		// Data logger
	STCCONTROLLER = 14,		// STCController
	STCDIAGNOSTIC = 15,		// STC Diagnostic
	STCSIMULATOR  = 21		// CU Simulator
}STCAPPID;

// Log record format
typedef struct _logm {
	char		LogTime[12];	// ssssss,mmm
	LOGLEV		LogLev;
	STCAPPID	LogApp;
	UINT		LogErrCode;
	char		LogMsg[120];
}LOGMSG;

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
typedef enum _stype {			// UNIHEAD Solution Status (table 7-118 and 7-49 of R1)
	SOL_COMPUTED = 0,			// Solution computed
	INSUFFICIENT_OBS = 1,		// Insufficient observation
	NO_CONVERGENCE = 2,			// No convergence, invalid solution
	COV_TRACE = 4				// Covariance matrix trace exceeds maximum (trace > 1000 m)
}SOLSTAT;

typedef enum _hstat {			// AGRICA Heading Status (table 7-80 of R1)
	INVALID = 0,				// Invalid solution
	FIXED   = 1,				// Fixed solution
	FLOAT   = 2,				// Float solution
}HEADSTAT;

typedef struct _gpsdata {
	char     UTCtime[12];
	SOLSTAT  SolStat;
	HEADSTAT HeadStat;
	float    latitude;
	char     latd;
	float    longitude;
	char     lond;
	float    Speed;
	float	 Heading;
	float	 Pitch;
	float	 Roll;
	float	 Altitude;
}GNSS;

typedef struct _envdata {
	UINT     Pressure;			// (hPa)
	float    Temperature;		// (Kelvin)
	UINT     Humidity;			// (%)
}ENV;

/* -------------------------------------------------------------------------------------------------------
   All UDP data
------------------------------------------------------------------------------------------------------- */
#include <arpa/inet.h>

typedef struct {
	struct sockaddr_in CUSideAddr;		// CU side address
	struct sockaddr_in STSideAddr;		// ST side address
	int    	  LstnSd;					// Listener socket
	int    	  TalkSd;					// Talker socket
	char	  STNetCard[16];			// The ST NetCard
	char	  STIPAddress[16];			// The ST IP Address
	UINT	  STUDPPort;				// The ST UDPlistening port
	int    	  CUsockfd;
	char	  CUIPAddress[16];			// The other IP Address
	UINT	  CUUDPPort;				// The other UDPlistening port
	bool	  CUIsConnected;			// The other is connected/recognized?
}UDPCOM;

/* -------------------------------------------------------------------------------------------------------
   The Self-Level data
------------------------------------------------------------------------------------------------------- */
typedef struct _sldata {
	bool LevelingState;
	bool MotorState[4];
	BYTE FWVersion;
	BYTE Errors[4];
	float XAxis;
	float YAxis;
}SLDATA;
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
#endif /* TYPES_H_ */

