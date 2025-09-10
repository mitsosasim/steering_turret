/* =======================================================================================================
  STCConfig.h

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
 ------------------------------------------------------------------------------


======================================================================================================= */
#ifndef CONFIG_H_
#define CONFIG_H_

#define CFGRECLEN 128
#define RECLABLEN  24	// for section, too
#define RECVALLEN  64

#include "STCTypes.h"

//#define STCDBG

typedef struct  _sys {					// SYSTEM
	int   			LogLevel;			// logs msg having level <= LogLevel (0 = disabled)
	int  			BitPeriod;			// perod to check devices health status
	bool  			TestPort;			// test port enabled/disabled
	UINT			UDPPort;			// ST Local port (ST receives here)
}SYSCFG;

typedef struct  _sfy {					// SAFETY
	bool			PhisycalBarrier;	// Enable/disable physical barrier alarm
	bool			RadarSensor;		// Enable/disable ultrasonic sensor alarm
	bool			Buzzer;				// Enable/disable the buzzer
	bool			BlinkLamp;			// Enable/disable the lamp
	int				MoveDelay;			// Delay time before enabling moving

}SFYCFG;

typedef struct  _cu {					// CONTROL UNIT
	char  			Address[16];		// CU authorized address
	UINT  			UDPPort;			// CU Local port (CU receives here)
	int	  			UpdTime;			// Telemetry update period
	int				Timeout;			// N secs = enable / 0 = disable CU timeout
}CUCFG;

typedef struct  _rs {					// Dual GPS antenna
	char			RSPort[64];
	ULONG			RSBaudRate;
	UINT			RSDataBit;
	char			RSParity;
	UINT			RSStopBit;
}RSCFG;

typedef struct _test {
	bool			AllowAnyAddr;		// Allow connection from any (e.g. Diagnostic)
}TESTCFG;

typedef struct  _stccfg {
	SYSCFG			sys;
	SFYCFG			safety;
	CUCFG			cu;
	RSCFG			sl;
	TESTCFG			test;
}STCCFG;

void CFGSetDefault();			// Set default configuration
int  CFGReadFile(char *);		// Read the config file
void CFGGetCurFile(char *);		// Just to know which is the current cfg file

void CFGGetSystem(STCCFG *);	// Return the whole system configuration
void CFGGetSafety(SFYCFG *);	// Return the SFY configuration
void CFGGetSyscfg(SYSCFG *);	// Return the SYS section configuration
void CFGGetSLcfg(RSCFG *);		// Return the SL section configuration
void CFGGetCUcfg(CUCFG *);		// Return the CU section configuration
void CFGGetTestcfg(TESTCFG *);	// Return the Testsection configuration

#endif /* CONFIG_H_ */

