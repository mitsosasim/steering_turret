/*==============================================================================
  NMEA.h

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine

 ------------------------------------------------------------------------------
   v. 1.0 - 01/12/2024
 ------------------------------------------------------------------------------
   Reference:
     R1. NebulasIV - High Precision Products - Rev 1.2 2023-11
==============================================================================*/
#ifndef NMEA_H_
#define NMEA_H_

#define SHMKEY 1234
#define SEMKEY 1234
#define SNAME "/mysem"

#define MAXNMEALEN 512
#define NMEADBG 1

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

typedef enum _nmeatype {
	NMEAUART = 1,				// NMEA from Serial Port
	NMEAFILE,					// NMEA form file (test scope)
	NMEACAN						// NMEA from CAN bus
}NMEASOURCETYPE;

typedef struct _smgpsdata {
	char     UTCtime[10];
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
}SMGNSS;

typedef struct _gnssp {
	char     UTCtime[10];
	SOLSTAT  SolStat;
	HEADSTAT HeadStat;
	char     GPSQuality;
	float    latitude;
	char     latd;
	float    longitude;
	char     lond;
	int      NumOfSV;
	float    Speed;
	float    Course ;
	float    LatSigmaErr;
	float    LonSigmaErr;
	float    HeiSigmaErr;
}GNSS_P;

typedef struct _gnssh {
	float Heading;
	float HeadStDev;
	float Pitch;
	float Roll;
	float Altitude;
}GNSS_H;

typedef struct _gnsst {
	unsigned int year;
	unsigned int month;
	unsigned int day;
	unsigned int hour;
	unsigned int minute;
	unsigned int second;
	char  		 GDO[20];
}GNSS_T;

typedef struct _nmeasrc {
	NMEASOURCETYPE		SrcType;
	char				SrcName[64];
	unsigned long int	RSBaudRate;
	unsigned int		RSDataBit;
	char				RSParity;
	unsigned int		RSStopBit;
}NMEASOURCE;

/* -------------------------------------------------------------------------------------------------------
   Public services
------------------------------------------------------------------------------------------------------- */
void NMEAStartThread(NMEASOURCE *);
void NMEAStopThread();
void NMEAGetPosition(GNSS_P *);
void NMEAGetHeading(GNSS_H *);
void NMEAGetDateTime(GNSS_T *);
int  NMEAGetThread(void);
int  NMEAThread(void *p);

void NMEASetDbgMode(int ShowReportFlag,		// +r
					int ShowMessageFlag,	// +m
					char *MessageFile,		// /M message file
					char *GPSDataFile);		// /R report file

/* -------------------------------------------------------------------------------------------------------
   Local functions (static)
------------------------------------------------------------------------------------------------------- */
//static int  nmeasrcopen();
//static void nmeasrcclose();
//static void nmeaprint();
//static void nmeaparse(char *);
//static int  nmearsread(char *);
//static int  nmeafileread(char *);
//static int  NMEAThread(void *);
#endif /* NMEA_H_ */
