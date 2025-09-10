/* =======================================================================================================
  STCGPSData.c

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine

 ------------------------------------------------------------------------------
   v. 1.0 - 01/12/2024 - Manage GPS data from Unicore UM892
 ------------------------------------------------------------------------------
   Reference:
     R1. NebulasIV - High Precision Products - Rev 1.2 2023-11
 ------------------------------------------------------------------------------
 UM982 Unicore’s high precision GNSS boards

 UARTx: 230400,8,N,1 (/dev/ttyUSBx) (for UM982)
 UARTx: 9600,8,N,1 (dev/ttyUSBx) (for PEDROS's single GPS receiver)

======================================================================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>	// queue
#include <sys/shm.h>	// shared memory
#include <sys/sem.h>	// semaphore

#include <sys/stat.h>
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()

#include "STCDefs.h"
#include "STCTypes.h"
#include "STCConfig.h"
#include "STCErrors.h"

#include "BCounter.h"
#include "STCShData.h"
#include "STCGPSData.h"

static SMGNSS LocData;
static GPSCFG GPSCfg;
static int hRSPort;

// Data structures
static GNSS_T   dtg;
static LOGMSG   eLog;

/* -------------------------------------------------------------------------------------------------------
   params:
   - cfg file
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	char   *whoami;
	char   cfgfile[128];

	// -------------------------------------------------------------------
    time_t nowtime;
    struct tm * timeinfo;
    time(&nowtime);
    timeinfo = localtime(&nowtime);

	// -------------------------------------------------------------------
	// I P C - Initialization
	// -------------------------------------------------------------------
	if(IPCInitialize() < 0) {
    	printf("STCGPSData E!. IPC initialization error\nExit\n");
        exit(1);
	}

	// -------------------------------------------------------------------
    // Program initialization
	// -------------------------------------------------------------------
	(whoami = strrchr(argv[0], '/')) ? ++whoami : (whoami = argv[0]);
    eLog.LogApp = STCGPSDATA;
	eLog.LogLev = INFOLOG;
	eLog.LogErrCode = STCNOERROR;
	sprintf(eLog.LogTime,"%02d%02d%02d,---",
			timeinfo->tm_hour,
			timeinfo->tm_min,
			timeinfo->tm_sec);
	sprintf(eLog.LogMsg,"%s[%d] v%d.%d.%c (build %04d)",
			whoami,eLog.LogApp,
			VERSION,RELEASE,DRAFT,BUILDNUMBER);
	IPCLogErrorS(&eLog);

	// -------------------------------------------------------------------
	printf("%s\n",eLog.LogMsg);

	// -------------------------------------------------------------------
	// Work environment and initial configuration
	// -------------------------------------------------------------------
 	CFGSetTree(whoami);
	CFGSetDefault();
	InitLocalData();

	// -------------------------------------------------------------------
	// Work with current/default config file
	// -------------------------------------------------------------------
	if(argc == 1) {
		CFGGetCurFile(cfgfile);
		//printf("STCGPSData: W!. Config file not specified. Default [%s] used\n",cfgfile);
		if(CFGReadDefault() != 0) {
			printf("STCGPSData: I!. Created the default cfg file\n");
		}
	}
	else if(argc == 2) {
		strcpy(cfgfile,argv[1]);
		if(CFGReadFile(cfgfile) != 0) {
			printf("STCGPSData: E!. Error in cfg file [%s]\nExit\n",cfgfile);
			exit(1);
		}
	}
	else {
		printf("STCGPSData: W!. Wrong arguments. Use default\n");
	}

	// ===================================================================
	// get current GPS configuration
	// -------------------------------------------------------------------
	CFGGetGPScfg(&GPSCfg);

	// -------------------------------------------------------------------
	if(GPSCfg.SrcType == NMEAUART) {
		printf("STCGPSData: I!. Read from UART [%s]\n",GPSCfg.SrcName);
		// -------------------------------------------------------------------
		if(CheckRSParams() != 0) {
			printf("STCGPSData: E!. Wrong RS setting.\nExit\n");
			exit(1);
		}
		if(SetRSPort() != 0) {
			printf("STCGPSData: E!. Error setting RS port.\n");
			printf("                check port availability\nExit\n");
			exit(1);
		}
		if(ReadGPSData() != 0)
			printf("STCGPSData: W!. Error reading from GPS device\n");
	}

	// -------------------------------------------------------------------
	else if(GPSCfg.SrcType == NMEAFILE) {
		struct stat st;
		printf("STCGPSData: I!. Read from FILE [%s]\n",GPSCfg.SrcName);
		if(stat(GPSCfg.SrcName,&st) == -1) {
			printf("STCGPSData: E!. [%s] does not exist.\nExit\n",GPSCfg.SrcName);
			exit(1);
		}
		if(ReadGPSFile() != 0)
			printf("STCGPSData: W!. Error reading from NMEA file\n");
	}
	// -------------------------------------------------------------------
	else if(GPSCfg.SrcType == NMEACAN) {
		printf("STCGPSData: I!. Read from CAN [%s]\n",GPSCfg.SrcName);
	}
	else {
		printf("STCGPSData: E!. Data source [%s] not valid\n",GPSCfg.SrcName);
		exit(1);
	}

	exit(0);
}

/* -------------------------------------------------------------------------------------------------------
   Infinite loop to read data from a NMEA file source
------------------------------------------------------------------------------------------------------- */
void InitLocalData()
{
	strcpy(LocData.UTCtime,"no data");
	LocData.SolStat		= INSUFFICIENT_OBS;
	LocData.HeadStat	= INVALID;
	LocData.latitude	= 0.00f;
	LocData.latd		= '-';
	LocData.longitude	= 0.00f;
	LocData.lond		= '-';
	LocData.Speed		= 0.00f;
	LocData.Heading		= 0.00f;
	LocData.Pitch		= 0.00f;
	LocData.Roll		= 0.00f;
	LocData.Altitude	= 0.00f;
}

/* -------------------------------------------------------------------------------------------------------
   Infinite loop to read data from a NMEA file source
------------------------------------------------------------------------------------------------------- */
int ReadGPSFile()
{
	char NMEAbuffer[MAXNMEALEN];

	FILE *hSrcf;
	int Result = 0;

	if((hSrcf = fopen(GPSCfg.SrcName,"r")) == NULL)
    	Result = 1;
	else {
		printf("STCGPSData: I!. start reading file [%s]\n",GPSCfg.SrcName);
		while(1) {
			if(fgets(NMEAbuffer,sizeof(NMEAbuffer),hSrcf) == NULL) {
				fseek(hSrcf,0,SEEK_SET);
				printf("STCGPSData: I!. File reload ...\n");
			//	fclose(hSrcf);
			//	hSrcf = NULL;
			//	printf("\n\nSource file completed.\n\n");
			//	break;
			}
			//printf("%s",NMEAbuffer);
			NMEAParse(NMEAbuffer);

			// Emulate 1 Hz GPS data
			if(strncmp(NMEAbuffer,"$GNGGA",6)==0)
				sleep(1);

			// Set global data
			IPCSetGPSData(&LocData);

		}
		fclose(hSrcf);
	}
	return(Result);
}

/* -------------------------------------------------------------------------------------------------------
   Infinite loop to read data from Unicore UM892
------------------------------------------------------------------------------------------------------- */
int ReadGPSData()
{
	int error = 0;
	char NMEAbuffer[MAXNMEALEN];

	memset(NMEAbuffer,'\0',sizeof(NMEAbuffer));
	do {
		if(NMEARSRead(NMEAbuffer) == 0)				// nothing read
			break;
		NMEAParse(NMEAbuffer);						// parse del message
		IPCSetGPSData(&LocData);					// assign global data
	}while(1);

	return(error);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int  CheckRSParams()
{
	int  BaudOK = 0;
	UINT baudrates[] = {   2400,   4800,   9600,  19200,  38400,  57600,
		                 115200, 230400, 460800, 500000, 576000, 921600 };

	// ------------------------------------------------------
	// check baudrate
	// ------------------------------------------------------
	for(int i=0;i<12;i++) {
		if(GPSCfg.RSBaudRate == baudrates[i]) {
			BaudOK = 1;
			break;
		}
	}
	if(!BaudOK)
		return(1);
	// ------------------------------------------------------
	// check databit
	// ------------------------------------------------------
	if(GPSCfg.RSDataBit != 8)
		return(1);
	// ------------------------------------------------------
	// check parity
	// ------------------------------------------------------
	if(GPSCfg.RSParity != 'N')
		return(1);
	// ------------------------------------------------------
	// check topbit
	// ------------------------------------------------------
	if(GPSCfg.RSStopBit != 1)
		return(1);
	return(0);
}

/* -------------------------------------------------------------------------------------------------------
   Open the RS NMEA source
------------------------------------------------------------------------------------------------------- */
int  SetRSPort()
{
	int    result = 0;
	struct termios tty;

	// ------------------------------------------------------
	// Open the RS port
	// ------------------------------------------------------
	hRSPort = open(GPSCfg.SrcName,O_RDWR);
	if(hRSPort < 0) {
		result = errno;
		//printf("STCGPSData:SetRSPort: E!. Error %d on open()\n",errno);
		return(result);
	}
	//printf("STCGPSData:SetRSPort: I!. open() OK=%d\n",hRSPort);

	// ------------------------------------------------------
	// Get curret RS settings
	// ------------------------------------------------------
	tcgetattr(hRSPort,&tty);

	// ------------------------------------------------------
	if(GPSCfg.RSParity == 'N')
		tty.c_cflag &= ~PARENB; 	// Clear parity bit, disabling parity (most common)
	else
		tty.c_cflag |= PARENB;  	// Set parity bit, enabling parity

	if(GPSCfg.RSStopBit == 1)
		tty.c_cflag &= ~CSTOPB; 	// Clear stop field, only one stop bit used in communication (most common)
	else
		tty.c_cflag |= CSTOPB;  	// Set stop field, two stop bits used in communication

	tty.c_cflag &= ~CSIZE; 			// Clear all the size bits, then use one of the statements below
	if(GPSCfg.RSDataBit == 8)
		tty.c_cflag |= CS8; 		// 8 bits per byte (most common)
	else if(GPSCfg.RSDataBit == 7)
		tty.c_cflag |= CS7; 		// 7 bits per byte
	else if(GPSCfg.RSDataBit == 6)
		tty.c_cflag |= CS6; 		// 6 bits per byte
	else if(GPSCfg.RSDataBit == 5)
		tty.c_cflag |= CS5; 		// 5 bits per byte

	tty.c_cflag &= ~CRTSCTS; 		// Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL;	// Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; 			// Disable echo
	tty.c_lflag &= ~ECHOE; 			// Disable erasure
	tty.c_lflag &= ~ECHONL; 		// Disable new-line echo
	tty.c_lflag &= ~ISIG; 			// Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; 			// Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; 			// Prevent conversion of newline to carriage return/line feed
	tty.c_cc[VMIN] = 0;				// wait for (at least) N characters (0 = do not wait)
	tty.c_cc[VTIME] = 10;    		// Wait for up to 1s (10 deciseconds), returning as soon as any data is received.

	// ------------------------------------------------------
	// Set in/out baudrate by calling cfsetispeed() / cfsetospeed()
	// ------------------------------------------------------
	switch(GPSCfg.RSBaudRate) {
		case 110:
			cfsetispeed(&tty,B110);
			break;
		case 300:
			cfsetispeed(&tty,B300);
			break;
		case 1200:
			cfsetispeed(&tty,B1200);
			break;
		case 2400:
			cfsetispeed(&tty,B2400);
			break;
		case 4800:
			cfsetispeed(&tty,B4800);
			break;
		case 9600:
			cfsetispeed(&tty,B9600);
			break;
		case 19200:
			cfsetispeed(&tty,B19200);
			break;
		case 38400:
			cfsetispeed(&tty,B38400);
			break;
		case 57600:
			cfsetispeed(&tty,B57600);
			break;
		case 115200:
			cfsetispeed(&tty,B115200);
			break;
		case 230400:
			cfsetispeed(&tty,B230400);
			break;
		case 460800:
			cfsetispeed(&tty,B460800);
			break;
		case 500000:
			cfsetispeed(&tty,B500000);
			break;
		case 576000:
			cfsetispeed(&tty,B576000);
			break;
		case 921600:
			cfsetispeed(&tty,B921600);
			break;
	}

	// ------------------------------------------------------
	// Finally try setting the new configuration
	// errno = 5 means this is a wrong port; try another
	// ------------------------------------------------------
	if(tcsetattr(hRSPort,TCSANOW, &tty) != 0) {
		close(hRSPort);
		result = errno;
		//printf("STCGPSData:SetRSPort: E!. Error %d on tcsetattr().\n",errno);
		return(result);
	}

	return(result);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int CheckCRC(char *msg)
{
	int  result = 0,
		 L,Q,j,temp;
	char buffcrc[5],
		 checkcrc[5];

    unsigned char chk = 0;
    const char *n = msg + 1; 			// Plus one, skip '$'

	L = strlen(msg);
	if(L >= (MAXNMEALEN-1) || L < 6)
		return(-1);						// Too long or too short

	// ------------------------------------------------------
	// This is the CRC included at the end of the NMEA
	//  sentence (after '*' character)
	// ------------------------------------------------------
    buffcrc[0]=msg[L-4];
    buffcrc[1]=msg[L-3];

	// ------------------------------------------------------
	// Recalculate the CRC included in the sentence.
    //   Skip '$' and '*'. End when newline
	// ------------------------------------------------------
    while('*' != *n && '\0' != *n) {
		if('\0' == *n || (n-msg) > MAXNMEALEN)
			return(-1);					// Sentence too long or short
		chk ^= (unsigned char)*n;
		n++;
    }

  	// ------------------------------------------------------
  	// This is the correct CRC for the received sentence
  	// ------------------------------------------------------
	j=0;
	Q=chk;
	while(Q != 0) {
		temp = Q % 16;
		if(temp < 10) temp += 48;
		else temp+=55;
		checkcrc[j++] = temp;
		Q /= 16;
	}

	// ------------------------------------------------------
	// CRC error
	// ------------------------------------------------------
	if((checkcrc[0] == buffcrc[0]) &&
		(checkcrc[1] == buffcrc[1])) {
		 //printf("CRC error %0x %0x vs %0x %0x\n",
		 //		checkcrc[0],buffcrc[1],
		 //		checkcrc[0],buffcrc[1]);
		return(-1);						// CRC error
	}

     return(result);
}

/* -------------------------------------------------------------------------------------------------------
   Read a NMEA stream from RS port
   return:
   n > 0 : number of character read
   0     : some timeout (no data received) when 100 missing data
------------------------------------------------------------------------------------------------------- */
int NMEARSRead(char *pBuffer)
{
	int  limit = MAXNMEALEN-1,
		 N,nb,TOut;
	char RSByte;
	char buff[MAXNMEALEN];

	// ------------------------------------------------------
	// Ignore all characters until '$' or '#'
	// Timeout break after 100 null reads: return 0
	// ------------------------------------------------------
	TOut=0;
	do {
		nb = read(hRSPort,&RSByte,1);
		if(nb == 0) {
			if(++TOut > 100)
				break;					// Timeout
		}
		else
			TOut = 0;
	}while(RSByte != '$' && RSByte != '#');

	// ------------------------------------------------------
	// Timeout on NMEA identifier
	// ------------------------------------------------------
	if(TOut > 0)
		return(0);

	// ------------------------------------------------------
	// Read all character until_
	// - exceeds max NMEA lenght: return -1
	// - (CR)/LF character: regular end, return num of bytes
	// ------------------------------------------------------
	buff[N=0] = RSByte;	// '$' or '#' is the first byte
	TOut=0;
	do {
		nb = read(hRSPort,&RSByte,1);
		if(nb == 0) {
			if(++TOut > 100)
				break;					// Timeout
		}
		else {
			TOut = 0;
			// printf("%c",RSByte);
			buff[++N] = RSByte;
			if(RSByte < 13)				// CR/LF = regular end
				break;
		}
	} while(N < limit);

	// ------------------------------------------------------
	// Timeout on NMEA character
	// ------------------------------------------------------
	if(TOut > 0)
		return(0);

	// ------------------------------------------------------
	// Message complete
	// ------------------------------------------------------
	buff[++N] = '\0';

	// ------------------------------------------------------
	// Check the CRC
	// ------------------------------------------------------
	if(CheckCRC(buff) != 0)
		return(-1);						// CRC error

	// ------------------------------------------------------
	// Regular exit
	// ------------------------------------------------------
	memcpy(pBuffer,&buff[0],N);
	return(N);
}

/* -------------------------------------------------------------------------------------------------------
 Manages only following NMEA-0183 sentences:
  $GNGGA $GNGSA $GNRMC $GNGST
 Manages following proprietary sentences:
  #UNIHEADINGA #AGRICA
------------------------------------------------------------------------------------------------------- */
void NMEAParse(char *pBuffer)
{
	static bool CheckOnce = FALSE;

	int   FieldId = 0;
	char  *pt,nmeatag[16];
	float sVal;
	int	  iVal;
	char  cVal;

	// ------------------------------------------------------
    pt = strtok(&pBuffer[0],",");
    if(pt == NULL || strlen(pt)>16) {
    	return;
    }

	// ------------------------------------------------------
	strcpy(nmeatag,pt);
	//printf("NMEAParse(%s)\n",nmeatag);

	/* ------------------------------------------------------
	0 	Message ID $GNGGA
	1 	UTC of position fix
	2 	Latitude
	3 	Direction of latitude:
		N: North
		S: South
	4 	Longitude
	5 	Direction of longitude:
		E: East
		W: West
	6 	GPS Quality indicator:
		0: Fix not valid
		1: GPS fix
		2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
		3: Not applicable
		4: RTK Fixed, xFill
		5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
		6: INS Dead reckoning
	7 	Number of SVs in use, range from 00 through to 24+
	8 	HDOP
	9 	Orthometric height (MSL reference)
	10 	M: unit of measure for orthometric height is meters
	11 	Geoid separation
	12 	M: geoid separation measured in meters
	13 	Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
	14 	Reference station ID, range 0000 to 4095. A null field when any reference station ID is selected and no corrections are received. See table below for a description of the field values.
	15 	The checksum data, always begins with *
	---------------------------------------------------------*/
	if(!strcmp(nmeatag,"$GNGGA")) {

	    pt = strtok(NULL,",");	// Field 1
	    FieldId++;				// NMEA 0183 counts from 0

	    while(pt != NULL) {
	    	if(FieldId == 1) {
	    		strcpy(LocData.UTCtime,pt); }
	    	else if(FieldId == 2) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.latitude = sVal; }
	    	else if(FieldId == 3) {
	    		cVal = pt[0];
	    		if(cVal == 'N' || cVal == 'S')
	    			LocData.latd = cVal; }
	    	else if(FieldId == 4) {
	    		sVal = atof(pt);
	    		if(sVal == 'E' || cVal == 'W')
	    			LocData.longitude = sVal; }
	    	else if(FieldId == 5) {
	    		cVal = pt[0];
	    		if(cVal > 0)
	    			LocData.lond = cVal; }
	    	else if(FieldId == 6) {
	    	//	cVal = pt[0];
	    	//	if(cVal > 0)
	    	//		pos.GPSQuality = cVal;
	    	}
	    	else if(FieldId == 7) {
	    	//	iVal = atoi(pt);
	    	//	if(iVal > 0)
	    	//		pos.NumOfSV = iVal;
	    	}
	        pt = strtok (NULL,",");
	    	++FieldId;
	    }
	}
	/* ------------------------------------------------------
	0 	Message ID $GNGSA
	1 	Mode 1:

	M = Manual
	A = Automatic
	2 	Mode 2: Fix type:

	1 = not available
	2 = 2D
	3 = 3D
	3 	PRN number:

	01 to 32 for GPS
	33 to 64 for SBAS
	64+ for GLONASS
	4 	PDOP: 0.5 to 99.9
	5 	HDOP: 0.5 to 99.9
	6 	VDOP: 0.5 to 99.9
	7 	The checksum data, always begins with *
	---------------------------------------------------------*/
	else if(!strcmp(nmeatag,"$GNGSA")) {
	}
	/* ------------------------------------------------------
	0 	Message ID $--RMC

	Talker ID can be:
	GP: GPS only
	GN: More than one constellation
	1 	UTC of position fix
	2 	Status A=active or V=void
	3 	Latitude
	4 	Direction of latitude:

	N: North
	S: South
	5 	Longitude
	6 	Direction of longitude:

	E: East
	W: West
	7 	Speed over the ground in knots
	8 	Track angle in degrees (True)
	9 	Date
	10 	Magnetic variation, in degrees
	11	The checksum data, always begins with *
	---------------------------------------------------------*/
	else if(!strcmp(nmeatag,"$GNRMC")) {

	    pt = strtok(NULL,",");	// Field 1
	    FieldId++;				// NMEA 0183 counts from 0

	    while(pt != NULL) {
	    	if(FieldId == 1) {
	    		strcpy(LocData.UTCtime,pt); }
	    	else if(FieldId == 3) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.latitude = sVal; }
	    	else if(FieldId == 4) {
	    		cVal = pt[0];
	    		if(cVal > 0)
	    			LocData.latd = cVal; }
		    else if(FieldId == 5) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.longitude = sVal; }
			else if(FieldId == 6) {
	    		cVal = pt[0];
	    		if(cVal > 0)
	    			LocData.lond = cVal; }
			else if(FieldId == 7) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.Speed = sVal; }
	    	else if(FieldId == 8) {
	    	//	iVal = atoi(pt);
	    	//	if(iVal > 0)
	    	//		pos.Course = iVal;
	    	}
	        pt = strtok (NULL,",");
	    	++FieldId;
	    }
	}
	/* ------------------------------------------------------
	0 	Message ID $GPGST
	1 	UTC of position fix
	2 	RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
	3 	Error ellipse semi-major axis 1-sigma error, in meters
	4 	Error ellipse semi-minor axis 1-sigma error, in meters
	5 	Error ellipse orientation, degrees from true north
	6 	Latitude 1-sigma error, in meters
	7 	Longitude 1-sigma error, in meters
	8 	Height 1-sigma error, in meters
	9 	The checksum data, always begins with *
	---------------------------------------------------------*/
	else if(!strcmp(nmeatag,"$GNGST")) {

	    pt = strtok(NULL,",");	// Field 1
	    FieldId++;				// NMEA 0183 counts from 0

	    while(pt != NULL) {
	    	if(FieldId == 1) {
	    		strcpy(LocData.UTCtime,pt); }
	    	//else if(FieldId == 6) {
	    	//	sVal = atof(pt);
	    	//	if(sVal>0)
	    	//		pos.LatSigmaErr = sVal; }
	    	//else if(FieldId == 7) {
	    	//	sVal = atof(pt);
	    	//	if(sVal>0)
	    	//		pos.LonSigmaErr = sVal; }
	    	//else if(FieldId == 8) {
	    	//	sVal = atof(pt);
	    	//	if(sVal>0)
	    	//		pos.HeiSigmaErr = sVal; }
	        pt = strtok (NULL,",");
	    	++FieldId;
	    }
	}
	/* ------------------------------------------------------
	1 	Message ID #UNIHEADING (proprietary)
		provides the heading information of the receiver in motion
	    #UNIHEADINGA,90,GPS,FINE,2350,221165600,0,0,18,13;
	                     ^   ^
	                     GPS time quality indicator (FINE,UNKNOWN)
	2 	sol stat	See Tables 7-49, 7-50
	                SOL_COMPUTED		Solution computed
	                INSUFFICIENT_OBS	Insufficient observation
	                NO_CONVERGENCE		No convergence, invalid solution
	                COV_TRACE			Covariance matrix trace exceeds maximum
	3	pos stat	See Table 0-5
	4	lenght		Baseline length
	5	heading		Heading (0 to 360.0 degrees, = field 21 of #AGRICA)
	6	pitch		Pitch (± 90 degrees)
	7	reserved
	8	hdgstddev	Standard deviation of heading
	9	ptchstddev	Standard deviation of pitch
	10	stn id		Base station ID
	11	#SVs		Number of satellites tracked
	12	#solnSVs	Number of satellites used in solution
	13	#obs		Number of satellites above the elevation mask angle
	14	#multi		Number of satellites with L2 signal
	15	reserved
	16	ext sol stat					See Tables 7-87
	17	Galileo&BDS3 sig mask			See Tables 7-86
	18	GPS, GLONASS and BDS2 sig mask	See Tables 7-85
	19	The checksum data				32-bit CRC
	20	CR/LF
	-----------
	   See R1 § 7.3.47 and Table 7-49
	---------------------------------------------------------*/
	else if(!strcmp(nmeatag,"#UNIHEADINGA")) {

	    pt = strtok(NULL,";");	// Field 1( Header) ends with ';'
	    pt = strtok(NULL,",");	// Field 2
	    FieldId = 2;			// R1.§7.3.47 counts from 1

	    while(pt != NULL) {
			if(FieldId == 2) {
				iVal = atoi(pt);
	    		if(iVal>0)
	    			LocData.SolStat = iVal; }
			else if(FieldId == 5) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.Heading = sVal; }
	    	//else if(FieldId == 8) {
	    	//	sVal = atof(pt);
	    	//	if(sVal>0)
	    	//		head.HeadStDev = sVal; }
	        pt = strtok (NULL,",");
	    	++FieldId;
	    }

	    if(!CheckOnce) {
			printf("STCGPSData:NMEAParse: I!. tag [%s]. Reading from UM982 OK\n",nmeatag);
			CheckOnce = TRUE;
	    }
	}

	/* --------------------------------------------------------
	1 	Message ID #AGRICA (proprietary)
		Provides the receiver's position, velocity, heading, and
		baseline information
	    #AGRICA,90,GPS,FINE,2350,221160600,0,0,18,12;
	                ^   ^
	               GPS time quality indicator (FINE,UNKNOWN)
	..
	4	Year:0=2000...255=2255
	5	Month: 1..12
	6	Day: 1..31
	7	Hour: 0..23
	8   Minute: 0...59
	9	Seconds: 0..59
	...
	11  Heading Status
		0: invalid solution
		4: fixed solution
		5: float solution
	...
	21	Heading (0 to 360.0 degrees, = field 5 of #UNIHEADINGA)
	22	Pitch
	23	Roll
	...
	33	Height of the rover station
	...
	59	The checksum data				32-bit CRC
	60	CR/LF
	-----------
	   See R1 § 7.3.22 and Table 7-49
	---------------------------------------------------------*/
	else if(!strcmp(nmeatag,"#AGRICA")) {
		char dbg[64];

	    pt = strtok(NULL,";");	// Field 1( Header) ends with ';'
	    pt = strtok(NULL,",");	// Field 2
	    FieldId = 2;			// R1.§7.3.22 counts from 1

	    while(pt != NULL) {
	    	strcpy(dbg,pt);
	    	if(FieldId == 4) {
	    		dtg.year = 2000+atoi(pt); }
	    	else if(FieldId == 5) {
	    		dtg.month = atoi(pt); }
	    	else if(FieldId == 6) {
	    		dtg.day = atoi(pt); }
	    	else if(FieldId == 7) {
	    		dtg.hour = atoi(pt); }
	    	else if(FieldId == 8) {
	    		dtg.minute = atoi(pt); }
	    	else if(FieldId == 9) {
	    		dtg.second = atoi(pt); }

	    	else if(FieldId == 11) {
	    		iVal = atoi(pt);
				if(iVal>0)
					LocData.HeadStat = atoi(pt); }
	    	else if(FieldId == 21) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.Heading = sVal; }
	    	else if(FieldId == 22) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.Pitch = sVal; }
	    	else if(FieldId == 23) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.Roll  = sVal; }
	    	else if(FieldId == 33) {
	    		sVal = atof(pt);
	    		if(sVal>0)
	    			LocData.Altitude  = sVal; }
	        pt = strtok (NULL,",");
	    	++FieldId;
	    }
	}
}

