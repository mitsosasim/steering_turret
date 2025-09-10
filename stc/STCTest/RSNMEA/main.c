/*
 * main.c
 *
 *  Created on: Jan 22, 2025
 *      Author: alma
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <errno.h> 		// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()

#include "NMEA.h"

void help(void);
int  checkfileargs(NMEASOURCE *);

/* -------------------------------------------------------------------------------------------------------
   params:
	/s read from RS port: <rsport> <baudrate> <databit> <parity> <stopbit>
	/f read nema from <filename>
	+m display all NMEA streams
	+r display GPS main data

	/l log NMEA on <filename> (not with /f)
	/v log Data on <filename>

	-h display the help

	/s and /f are exclusive\n");
------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
	int  ShowMsg = 0;
	int  ShowReport = 0;
	char MsgLogFile[255] = { '\0' };
	char RepLogFile[255] = { '\0' };

	NMEASOURCE 	ThrdPrm;
	GNSS_P PosData;
	GNSS_H HeadData;
	GNSS_T DateTime;

	// ------------------------------------------------------
	int i;
	if(argc <= 1) {		// no parameters
		help();
		return(0);
	}
	else {
	    for(i=1;i<argc;++i) {
	        //printf("param %i, = %s\n",i,argv[i]);
	    	// ------------------------------------------------------
	        if(strcmp("-h",argv[i])==0) {
	        	help();
	        	return(0);
	        }
	    	// ------------------------------------------------------
	        else if(strcmp("+m",argv[i])==0) {
	        	ShowMsg = 1;
	        }
	    	// ------------------------------------------------------
	        else if(strcmp("+r",argv[i])==0) {
	        	ShowReport = 1;
	        }
	    	// ------------------------------------------------------
	        else if(strcmp("/s",argv[i])==0) {
	        	if(argc >= i+6) {
	        		ThrdPrm.SrcType    = NMEAUART;
	        		strcpy(ThrdPrm.SrcName,argv[++i]);
	        		ThrdPrm.RSBaudRate = atoi(argv[++i]);
	        		ThrdPrm.RSDataBit  = atoi(argv[++i]);
	        		ThrdPrm.RSParity   = *argv[++i];
	        		ThrdPrm.RSStopBit  = atoi(argv[++i]);
	        		checkfileargs(&ThrdPrm);
	        	}
	        	else {
	        		printf("/s insufficient arguments\n");
		        	return(1);
	        	}
	        }
	    	// ------------------------------------------------------
	        else if(strcmp("/f",argv[i])==0) {
	        	if(strlen(RepLogFile)) {
	        		printf("/F not allowed with /v\n");
		        	return(1);
	        	}
	        	if(argc >= i+2) {
	        		strcpy(ThrdPrm.SrcName,argv[++i]);
	        	    struct stat buffer;
	        	    if(stat(ThrdPrm.SrcName,&buffer) != 0) {
		        		printf("/f file <%s> does not exist\n",ThrdPrm.SrcName);
			        	return(1);
	        	    }
	        		ThrdPrm.SrcType = NMEAFILE;
	           	}
	        	else {
	        		printf("/f must be followed by a file name\n");
		        	return(1);
	        	}
		    }
	    	// ------------------------------------------------------
	        else if(strcmp("/v",argv[i])==0) {
	        	if(ThrdPrm.SrcType == NMEAFILE) {
	        		printf("/v not allowed with /f\n");
		        	return(1);
	        	}
	        	if(argc >= i+2)
	        		strcpy(RepLogFile,argv[++i]);
	        	else {
	        		printf("/v must be followed by a file name\n");
		        	return(1);
	        	}
		    }
	        else if(strcmp("/l",argv[i])==0) {
	        	if(argc >= i+2) {
	        		strcpy(MsgLogFile,argv[++i]);
	           	}
	        	else {
	        		printf("/l must be followed by a file name\n");
		        	return(1);
	        	}
		    }
        	else {
        		printf("<%s> unknown argument\n",argv[i]);
	        	return(1);
        	}

	    }
	}

	/* ------------------------------------------------------
	// Read from RS
	//ThrdPrm.SrcType = NMEARS;
	//strcpy(ThrdPrm.SrcName,RSPORT);
	ThrdPrm.RSBaudRate = RSBAUD;
	ThrdPrm.RSDataBit = RSDATABIT;
	ThrdPrm.RSParity = RSPARITY;
	ThrdPrm.RSStopBit = RSSTOPBIT;
	// Read from file
	ThrdPrm.SrcType = NMEAFILE;
	strcpy(ThrdPrm.SrcName,"capture1");
	------------------------------------------------------ */
	char  retdesc[128];
	printf("-------------------------------------------------\n");
	printf("Test GPS receiver\n");
	printf("-------------------------------------------------\n");
	if(getcwd(retdesc,sizeof(retdesc)))
		printf("Current working dir: %s\n", retdesc);

	// ------------------------------------------------------
	//  '/s' '/f' was specified
	// ------------------------------------------------------
	if(checkfileargs(&ThrdPrm) != 0)
		exit(1);

	// ------------------------------------------------------
	// read from file ('/f') or from RS ('/s')
	// ------------------------------------------------------
	NMEASetDbgMode(ShowMsg,ShowReport,MsgLogFile,RepLogFile);
	NMEAStartThread(&ThrdPrm);

	if(!NMEAGetThread())
		printf("- Failed to run!\n");
	else {
		printf("- NMEA thread running!\n");
		// ------------------------------------------------------
		//  '+m' '+r' or '/l' was requested
		// ------------------------------------------------------
		NMEASetDbgMode(ShowMsg,ShowReport,MsgLogFile,RepLogFile);
		do {
			sleep(1);
			if(!NMEAGetThread()) {
				printf("- NMEA thread stop!\n");
				break;
			}
			NMEAGetPosition(&PosData);
			NMEAGetHeading(&HeadData);
			NMEAGetDateTime(&DateTime);

		}while(1);

		NMEAStopThread();
		printf("- Test <%s> completed!\n",ThrdPrm.SrcName);
	}

	return 0;
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void help(void)
{
	printf("usage:\n");
	printf("./RSNMEA arguments:\n");
	printf("   /s read NMEA from RS port: <rsport> <baudrate> <databit> <parity> <stopbit>\n");
	printf("      (e.g) /s /dev/ttyUSB0 9600 8 N 1\n");
	printf("   /f read NMEA fron <filename>\n");
	printf("      (e.g) /f record.txt\n");
	printf("   /l log NMEA on <filename>\n");
	printf("      (e.g) /l messages.txt\n");
	printf("   /v extract lat/lon/head on on <filename>\n");
	printf("      (e.g) /v report.txt\n");
	printf("   +m display all NMEA streams\n");
	printf("   +r display GPS main data\n");
	printf("   /h display this help\n");
	printf(" note:\n");
	printf("   /s and /f are alternative\n");
	printf("   /v incompatible with /f\n\n");
}

/* -------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------
void rawtest()
{
	int			   hPort;
	struct termios tty;

	hPort = open("/dev/ttyUSB0",O_RDWR);
	if(hPort > 0) {
		printf("port /dev/ttyUSB0 OK\n");
		tcgetattr(hPort,&tty);

		tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

		if(cfsetispeed(&tty,B230400))
			printf("cfsetispeed error\n");
		if(tcsetattr(hPort,TCSANOW, &tty) != 0) {
			printf("tcsetattr error\n");
		}

		// ------------------------------------------------------
		// Ignore all characters until '$' or '#'
		// Timeout break after 100 null reads: return 0
		// ------------------------------------------------------
		int  nb,counter = 0;
		char RSByte;
		do {
			nb = read(hPort,&RSByte,1);
			printf("%c",RSByte);
		}while(++counter < 1000);

		close(hPort);

	}

} */


/* -------------------------------------------------------------------------------------------------------
   Check source parameter correctness. Returns:
   1 in case of errors
   0 everything looks correct
------------------------------------------------------------------------------------------------------- */
int  checkfileargs(NMEASOURCE *p)
{
	int  BaudOK = 0;
	FILE *hFp;
	unsigned long int baudrates[] = {   2400,   4800,   9600,  19200,  38400,  57600,
			                          115200, 230400, 460800, 500000, 576000, 921600 };

	// ----------------------------------------------------------------------------------------
	// /f file source must exist and be readable
	// ----------------------------------------------------------------------------------------
	if(p->SrcType == NMEAFILE) {
		printf("Source file: <%s>\n",p->SrcName);
		hFp=fopen(p->SrcName,"r");
		if(hFp == NULL) {
    		printf("/f file <%s> does not exist or permission error\n",p->SrcName);
        	return(1);
	    }
		fclose(hFp);
	}

	// ------------------------------------------------------
	// /s RS parameters settings
	// ------------------------------------------------------
	else if(p->SrcType == NMEAUART) {
		printf("Source file: <%s> %ld %d %c %d\n",
				p->SrcName,
				p->RSBaudRate,
				p->RSDataBit,
				p->RSParity,
				p->RSStopBit);

		// check baudrate
		for(int i=0;i<12;i++) {
			if(p->RSBaudRate == baudrates[i]) {
				BaudOK = 1;
				break;
			}
		}
		if(!BaudOK) {
			printf("baudrate %ld not allowed\n",p->RSBaudRate);
	    	return(1);
		}

		// check databit
		if(p->RSDataBit != 8) {
			printf("databit %d not allowed\n",p->RSDataBit);
	    	return(1);
		}

		// check parity
		if(p->RSParity != 'N') {
			printf("parity %c not allowed\n",p->RSParity);
	    	return(1);
		}

		// check topbit
		if(p->RSStopBit != 1) {
			printf("stopbit %d not allowed\n",p->RSStopBit);
	    	return(1);
		}
	}
	else {
		printf("A source file must be specified (use /s or /f)\n");
    	return(1);
	}

	return(0);
}
