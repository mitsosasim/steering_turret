/*===========================================================================
 Name        : STC_Config.c
 Author      : R. Filippone
 Version     :
 Copyright   : ALMA sistemi srl
 Description :
 ---------------------------------------
 Error codes:
	CFG: error 131 - record too long
	CFG: error 132 - record too short
	CFG: error 133 - label or value wrong size
	CFG: error 134 - wrong syntax
 	CFG: error 135 - unknown parameter
 	CFG: error 136 - unknown section
 ==========================================================================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <sys/stat.h>

#include "STCTypes.h"
#include "STCDefs.h"
#include "STCErrors.h"
#include "STCConfig.h"

// private attributes
static  STCCFG Config;
static  char CfgFileName[128];
static  int Initialized = 0,
			rownum;

// private members
static void writecfg();
static void readcfg();
static int  checkrow(char *cfgrow);

#ifdef STCDBG
static void showcfg();
#endif

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void CFGSetDefault()
{
	rownum = 0;

	// System
	Config.sys.LogLevel = 0;					// log info with level >=
	Config.sys.BitPeriod = 300;					// Check devices every..
	Config.sys.TestPort = false;				// Local /dev/tty port
	Config.sys.UDPPort = 9966;					// Local ST UDP port

	// Safety
	Config.safety.PhisycalBarrier = true;		// Physical barrier alarm: enabled
	Config.safety.RadarSensor = true;			// Radar sensor alarm: enabled
	Config.safety.Buzzer = true;				// Movement buzzer: enabled
	Config.safety.BlinkLamp = true;				// Movement light: enabled
	Config.safety.MoveDelay = 5;				// Enable moving after N seconds

	// Control Unit
	strcpy(Config.cu.Address,"192.168.1.18");	// 192.168.1.18
	Config.cu.UDPPort = 9914;					// UDP port
	Config.cu.UpdTime = 300;					// Send to CU every ....
	Config.cu.Timeout = 300;					// After 5 mins declares timeout

	// Self-Level system
	strcpy(Config.sl.RSPort,"/dev/ttyS0");
	Config.sl.RSBaudRate		= 115200;
	Config.sl.RSDataBit			= 8;
	Config.sl.RSParity			= 'N';
	Config.sl.RSStopBit			= 1;

	// test support
	Config.test.AllowAnyAddr	= false;

	Initialized = 1;
}

/* -------------------------------------------------------------------------------------------------------
   Read the specified config file.
   - if file does not exist, create one with default values
------------------------------------------------------------------------------------------------------- */
int CFGReadFile(char * cfgfile)
{
	int    result = 0; // no error
	struct stat st;

	// -----------------------------------------------------------
	if(!Initialized)
		CFGSetDefault();

	// -----------------------------------------------------------
	strcpy(CfgFileName,cfgfile);

	// -----------------------------------------------------------
	// Config file does not exist. Create one with default values
	// -----------------------------------------------------------
	if(stat(CfgFileName,&st) == -1) {
		writecfg();
		result = 1;
	}
	// -----------------------------------------------------------
	// Read the config file
	// -----------------------------------------------------------
	else {
		readcfg();
	}

#ifdef STCDBG
	showcfg();
#endif
	return(result);
}


/* -------------------------------------------------------------------------------------------------------
  Questo file viene letto solo una volta, all'inizio dell'esecuzione.
  Per evitare errori (overflow) nel formato del file (potenzialmente modificato dall'operatore)
  viene effettuato un controllo sintattico.
------------------------------------------------------------------------------------------------------- */
static void readcfg()
{
	FILE  *pHc;
	char  record[128];
	char  section[RECLABLEN],label[RECLABLEN],eq,value[RECVALLEN];
	UINT  uiValue,res;
	ULONG ulValue;
	char  cValue;

	// -----------------------------------------------------------
	pHc=fopen(CfgFileName,"r");
	while(pHc != NULL) {

		if(fgets(record,128,pHc) == NULL)
			break;

		rownum++;
		res = checkrow(record);
		// -------------------------------------------------------
		// Section
		// -------------------------------------------------------
		while(res == 1) {
			sscanf(record,"%s",section);
			if(fgets(record,128,pHc) == NULL)
				break;

			res = checkrow(record);
			// ---------------------------------------------------
			// Parameters
			// ---------------------------------------------------
			while(res == 3) {
				if(sscanf(record,"%s %c %s\n",label,&eq,value) != 3)
					break;

				// -----------------------------------------------
				// SECTION: [SYSTEM]
				// -----------------------------------------------
				if(!strcmp(section,"[SYSTEM]")) {
					// -------------------------------------------
					// Log level: the cutting log threshold
					// according following (see also ComDefs)
					// 0: info, warnings, errors, criticals
					// 1: warnings, errors, criticals
					// 2: errors, criticals
					// 3: only criticals
					// -------------------------------------------
					if(!strcmp(label,"LogLevel")) {
						uiValue = atoi(value);
						if(uiValue >= DISABLED && uiValue <= INFOLOG) {
							Config.sys.LogLevel = uiValue;
						}
					}
					// -------------------------------------------
					// BIT period: in mSecs
					// Time of device check (see ThCANOpen)
					// -------------------------------------------
					else if(!strcmp(label,"BITperiod")) {
						uiValue = atoi(value);
						if(uiValue >= 10 && uiValue < 5000) {
							Config.sys.BitPeriod = uiValue;
						}
					}
					// -------------------------------------------
					// Test port: when "enabled", the log will be 
					// (also) forward to the /dev/tty (see ThTestPort)
					// -------------------------------------------
					else if(!strcmp(label,"TestPort"))
						Config.sys.TestPort = strcmp(value,"enabled")?false:true;
					// -------------------------------------------
					// Local UDP port (Server side)
					// -------------------------------------------
					else if(!strcmp(label,"UDPPort")) {
						uiValue = atoi(value);
						if(uiValue >= 1024 && uiValue <= 49151) {
							Config.sys.UDPPort = uiValue;					// UDP port
						}
					}
					// -------------------------------------------
					// Unknown parameter: error code 135.
					// -------------------------------------------
					else {
						printf("CFG: syntax error line %d. unknown label [%s]\n",rownum,label);
						break;
					}
				}
				// -----------------------------------------------
				// SECTION: Safety
				// -----------------------------------------------
				else if(!strcmp(section,"[SAFETY]")) {
					// -------------------------------------------
					// Enable/Disable Phisycal Barrier alarm
					// -------------------------------------------
					if(!strcmp(label,"Barrier"))
						Config.safety.PhisycalBarrier = strcmp(value,"enabled")?false:true;
					// -------------------------------------------
					// Enable/Disable Radar Sensor alarm
					// -------------------------------------------
					else if(!strcmp(label,"RadarZone"))
						Config.safety.RadarSensor = strcmp(value,"enabled")?false:true;
					// -------------------------------------------
					// Enable/Disable buzzer alarm
					// -------------------------------------------
					else if(!strcmp(label,"Buzzer"))
						Config.safety.Buzzer = strcmp(value,"enabled")?false:true;
					// -------------------------------------------
					// Enable/Disable lamp alarm
					// -------------------------------------------
					else if(!strcmp(label,"Lamp"))
						Config.safety.BlinkLamp = strcmp(value,"enabled")?false:true;
					// -------------------------------------------
					// Enable/Disable moving N second delay after
					// starting Buzzer. If Buzzer if Off Delay is 0
					// -------------------------------------------
					else if(!strcmp(label,"Delay")) {
						uiValue = atoi(value);
						Config.safety.MoveDelay = uiValue;
					}
					// -------------------------------------------
					// Unknown parameter: error code 135.
					// -------------------------------------------
					else {
						printf("CFG: syntax error line %d. unknown label [%s]\n",rownum,label);
						break;
					}
				}
				// -----------------------------------------------
				// SECTION: Control Unit
				// -----------------------------------------------
				else if(!strcmp(section,"[CUNIT]")) {
					// -------------------------------------------
					// CU IP address: the IP address of the Control
					//  Unit, for UDP communications
					// -------------------------------------------
					if(!strcmp(label,"Address")) {
						if(strlen(value)<16) {
							strcpy(Config.cu.Address,value);
						}
					}
					// -------------------------------------------
					// CU IP address: the IP address of the Control
					//  Unit, for UDP communications
					// -------------------------------------------
					else if(!strcmp(label,"UDPPort")) {
						uiValue = atoi(value);
						if(uiValue >= 1024 && uiValue <= 49151) {
							Config.cu.UDPPort = uiValue;
						}
					}
					// -------------------------------------------
					// Control Unit update period: in mSecs.
					// Data will be sent to CU every CU update time
					// -------------------------------------------
					else if(!strcmp(label,"Update")) {
						uiValue = atoi(value);
						if(uiValue > 10 && uiValue < 10000) {
							Config.cu.UpdTime = uiValue;
						}
					}
					// -------------------------------------------
					// Enable/Disable timeout check: in Secs.
					// 0 = disabled
					// N = declares CU timeout after N seconds
					//     where ST does not receive data
					// -------------------------------------------
					else if(!strcmp(label,"Timeout")) {
						uiValue = atoi(value);
						if(uiValue >= 0 && uiValue <= 3600) {
							Config.cu.Timeout = uiValue;
						}
					}
					// -------------------------------------------
					// Unknown parameter: error code 135.
					// -------------------------------------------
					else {
						printf("CFG: syntax error line %d. unknown label [%s]\n",rownum,label);
						break;
					}
				}
				// -----------------------------------------------
				// SECTION: [Self-Levl]
				// -----------------------------------------------
				else if(!strcmp(section,"[SELFLEVEL]")) {
					if(!strcmp(label,"RSPort")) {
						strcpy(Config.sl.RSPort,value);
					}
					else if(!strcmp(label,"RSBaudRate")) {
						ulValue = atol(value);
						Config.sl.RSBaudRate = ulValue;
					}
					else if(!strcmp(label,"RSDataBit")) {
						uiValue = atoi(value);
						if(uiValue == 8)
							Config.sl.RSDataBit = uiValue;
					}
					else if(!strcmp(label,"RSParity")) {
						cValue = value[0];
						if(cValue == 'N')
							Config.sl.RSParity = cValue;
					}
					else if(!strcmp(label,"RSStopBit")) {
						uiValue = atoi(value);
						if(uiValue == 1)
							Config.sl.RSStopBit = uiValue;
					}
					// -------------------------------------------
					// Unknown parameter: error code 135.
					// -------------------------------------------
					else {
						printf("CFG: syntax error line %d. unknown label [%s]\n",rownum,label);
						break;
					}
				}
				// -----------------------------------------------
				// SECTION: [TEST]
				// -----------------------------------------------
				else if(!strcmp(section,"[TEST]")) {
					// -------------------------------------------
					// Other IP address: allow access from any IP
					// -------------------------------------------
					if(!strcmp(label,"AllowAnyAddr")) {
						Config.test.AllowAnyAddr = strcmp(value,"enabled")?false:true;
					}
				}
				// -----------------------------------------------
				// Unknown section: error code 136.
				// -----------------------------------------------
				else {
					printf("CFG: syntax error line %d. Unknown section %s\n",rownum,section);
				}

				if(fgets(record,128,pHc) == NULL)
					break;

				// -----------------------------------------------
				// next row
				// -----------------------------------------------
				rownum++;
				res = checkrow(record);

			}	// 3 parameters
		}	// section
	}	// EOF
	fclose(pHc);

	// -----------------------------------------------------------
	if(Config.safety.Buzzer == false)
		Config.safety.MoveDelay = 0;

}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
static void writecfg()
{
	FILE *pHc;
	char filepath[32];
	char sValue[24];
	time_t nowtime;
	struct tm * timeinfo;

	time(&nowtime);
	timeinfo = localtime(&nowtime);

	// -------------------------------------------------------------------
	sprintf(filepath,"%s/%s",CFGPATH,CFGFILE);
	pHc=fopen(filepath,"w");
	if(pHc != NULL) {
		fprintf(pHc,"# Config file for ST Turret control application\n");
		fprintf(pHc,"# ALMA sistemi srl. - Roma - Italy\n");
		fprintf(pHc,"# Last update: %02d.%02d.%4d-%02d:%02d\n",
			timeinfo->tm_mday,
			timeinfo->tm_mon,
			timeinfo->tm_year+1900,
			timeinfo->tm_hour,
			timeinfo->tm_min);

		// 	System
		fprintf(pHc,"\n[SYSTEM]\n");
		fprintf(pHc,"LogLevel = %d\n",Config.sys.LogLevel);
		fprintf(pHc,"BITperiod = %d\n",Config.sys.BitPeriod);
		if(Config.sys.TestPort == true)
			strcpy(sValue,"enabled");
		else
			strcpy(sValue,"disabled");
		fprintf(pHc,"TestPort = %s\n",sValue);
		fprintf(pHc,"UDPPort = %d\n",Config.sys.UDPPort);

		// 	Safety
		fprintf(pHc,"\n[SAFETY]\n");
		if(Config.safety.PhisycalBarrier == true)
			strcpy(sValue,"enabled");
		else
			strcpy(sValue,"disabled");
		fprintf(pHc,"Barrier = %s\n",sValue);

		if(Config.safety.RadarSensor == true)
			strcpy(sValue,"enabled");
		else
			strcpy(sValue,"disabled");
		fprintf(pHc,"RadarZone = %s\n",sValue);

		if(Config.safety.Buzzer == true)
			strcpy(sValue,"enabled");
		else
			strcpy(sValue,"disabled");
		fprintf(pHc,"Buzzer = %s\n",sValue);
		if(Config.safety.BlinkLamp == true)
			strcpy(sValue,"enabled");
		else
			strcpy(sValue,"disabled");
		fprintf(pHc,"Lamp = %s\n",sValue);
		fprintf(pHc,"Delay = %d\n",Config.safety.MoveDelay);

		// 	Control Unit
		fprintf(pHc,"\n[CUNIT]\n");
		fprintf(pHc,"Address = %s\n",Config.cu.Address);
		fprintf(pHc,"UDPPort = %d\n",Config.cu.UDPPort);
		fprintf(pHc,"Update = %d\n",Config.cu.UpdTime);
		fprintf(pHc,"Timeout = %d\n",Config.cu.Timeout);

		// 	SELFLEVEL device
		fprintf(pHc,"\n[SELFLEVEL]\n");
		fprintf(pHc,"RSPort = %s\n",Config.sl.RSPort);
		fprintf(pHc,"RSBaudRate = %ld\n",Config.sl.RSBaudRate);
		fprintf(pHc,"RSDataBit = %d\n",Config.sl.RSDataBit);
		fprintf(pHc,"RSParity = %c\n",Config.sl.RSParity);
		fprintf(pHc,"RSStopBit = %d\n",Config.sl.RSStopBit);

		// 	Test scopes section
		fprintf(pHc,"\n[TEST]\n");
		if(Config.test.AllowAnyAddr == true)
			strcpy(sValue,"enabled");
		else
			strcpy(sValue,"disabled");
		fprintf(pHc,"AllowAnyAddr = %s\n",sValue);

		fflush(pHc);
		fclose(pHc);
	}
}

/* -------------------------------------------------------------------------------------------------------
   Check the config file record correctness:

   record starting with # are comments: ignored
   empty records: ignored
   abel identifier longer than expected: ignored
   value field longer than expected: ignored

   record must be:
      [SECTION]			= 1 field => section identifier
      label = value		= 3 fields => parameter specification

   return:
	1 section
	3 parameter
	0 ignored record
------------------------------------------------------------------------------------------------------- */
static int checkrow(char *cfgrow)
{
	int    FuncCode = 0;
	char   *lab,*eqq,*val;

	char dbg[128];

	// -----------------------------------------
	size_t L = strlen(cfgrow);
	if(cfgrow[0] == '#' || L <= 1) {			// comment or empty row
		return(FuncCode);
	}
	if(L >= (RECLABLEN + 3 + RECVALLEN)) {		// too long record
		printf("CFG: syntax error line %d.\n",rownum);
		return(FuncCode);
	}
	if(L < 4) {									// too short record
		printf("CFG: syntax error line %d.\n",rownum);
		return(FuncCode);
	}

	// -----------------------------------------
	lab=(char *) malloc(CFGRECLEN);
	eqq=(char *) malloc(CFGRECLEN);
	val=(char *) malloc(CFGRECLEN);

	// -----------------------------------------
	FuncCode = sscanf(cfgrow,"%s %s %[^\n]s\n",lab,eqq,val);
	if(FuncCode == 3) {
		if(strlen(lab) >= RECLABLEN) {
			FuncCode = 0;
			printf("CFG: syntax error line %d. Ignored.\n",rownum);
		}
		if(strlen(eqq) != 1 || eqq[0] != '=') {
			FuncCode = 0;
			printf("CFG: syntax error line %d. Ignored.\n",rownum);
		}
		if(strlen(val) >= RECVALLEN) {
			FuncCode = 0;
			printf("CFG: syntax error line %d. Ignored.\n",rownum);
		}
	}
	else if (FuncCode == 1) {}
	else {
		FuncCode = 0;
		strcpy(&dbg[0],cfgrow);
		printf("CFG: syntax error line %d. Ignored.\n",rownum);
	}

	// -----------------------------------------
	free(val);
	free(eqq);
	free(lab);

	return(FuncCode);
}

#ifdef STCDBG
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
static void showcfg()
{
	char sValue[24];
	printf("File: [%s]\n\n",CfgFileName);

	printf("[SYSTEM]\n");
	printf("LogLevel = %d\n",Config.sys.LogLevel);
	printf("BITperiod = %d\n",Config.sys.BitPeriod);
	if(Config.sys.TestPort == true)
		strcpy(sValue,"enabled");
	else
		strcpy(sValue,"disabled");
	printf("TestPort = %s\n",sValue);
	printf("UDPPort = %d\n",Config.sys.UDPPort);

	printf("[SAFETY]\n");
	printf("\n[SAFETY]\n");
	if(Config.safety.PhisycalBarrier == true)
		strcpy(sValue,"enabled");
	else
		strcpy(sValue,"disabled");
	printf("Barrier = %s\n",sValue);

	if(Config.safety.RadarSensor == true)
		strcpy(sValue,"enabled");
	else
		strcpy(sValue,"disabled");
	printf("RadarZone = %s\n",sValue);

	if(Config.safety.Buzzer == true)
		strcpy(sValue,"enabled");
	else
		strcpy(sValue,"disabled");
	printf("Buzzer = %s\n",sValue);
	if(Config.safety.BlinkLamp == true)
		strcpy(sValue,"enabled");
	else
		strcpy(sValue,"disabled");
	printf("Lamp = %s\n",sValue);
	printf("Delay = %d",Config.safety.MoveDelay);

	printf("[CUNIT]\n");
	printf("Address = %s\n",Config.cu.Address);
	printf("UDPPort = %d\n",Config.cu.UDPPort);
	printf("Update = %d\n",Config.cu.UpdTime);
	printf("Timeout = %d\n",Config.cu.Timeout);

	printf("[SELFLEVEL]\n");
	printf("RSPort = %s\n",Config.sl.RSPort);
	printf("RSBaudRate = %ld\n",Config.sl.RSBaudRate);
	printf("RSDataBit = %d\n",Config.sl.RSDataBit);
	printf("RSParity = %c\n",Config.sl.RSParity);
	printf("RSStopBit = %d\n",Config.sl.RSStopBit);

	printf("[TEST]\n");
	if(Config.test.AllowAnyAddr == true)
		strcpy(sValue,"enabled");
	else
		strcpy(sValue,"disabled");
	printf("AllowAnyAddr = %s\n",sValue);

}
#endif

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void CFGGetCurFile(char *p)
{
	strcpy(p,CfgFileName);
}

/* -------------------------------------------------------------------------------------------------------
  Return the whole configuration
------------------------------------------------------------------------------------------------------- */
void CFGGetSystem(STCCFG *p)
{
	memcpy(p,&Config,sizeof(STCCFG));
}

/* -------------------------------------------------------------------------------------------------------
  Return the sys section configuration
------------------------------------------------------------------------------------------------------- */
void CFGGetSyscfg(SYSCFG *p)
{
	memcpy(p,&Config.sys,sizeof(SYSCFG));
}

/* -------------------------------------------------------------------------------------------------------
  Return the CU section configuration
------------------------------------------------------------------------------------------------------- */
void CFGGetCUcfg(CUCFG *p)
{
	memcpy(p,&Config.cu,sizeof(CUCFG));
}

/* -------------------------------------------------------------------------------------------------------
  Return the Safety section configuration
------------------------------------------------------------------------------------------------------- */
void CFGGetSafety(SFYCFG *p)
{
	memcpy(p,&Config.safety,sizeof(SFYCFG));
}

/* -------------------------------------------------------------------------------------------------------
  Return the CU section configuration
------------------------------------------------------------------------------------------------------- */
void CFGGetTestcfg(TESTCFG *p)
{
	memcpy(p,&Config.test,sizeof(TESTCFG));
}


