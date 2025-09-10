/*
 ============================================================================
 Name        : ClevBuild.c
 Author      : R. Filippone
 Version     :
 Copyright   : ALMA sistemi srl
 Description :
 ============================================================================
 */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

#include "ClevBuild.h"

int   BuildNum = 0;
char  DataRows[BCROWS][HRECLEN];

/*--------------------------------------------------------------------
  ClevBuild riceve in input (prm) la cartella di lavoro (tipicamente
    la cartella dove risiedono i file header (.h) del progetto.
  ClevBuild cerca in questa cartella il file BCounter.txt. Questo file
    contiene il nome del file .h che sarà aggiornato con le definizioni
    di build e che sarà compilato con il progetto.
  Se il file BCounter.txt non esiste, sarà creato un file con valori di
    default:
    header file = BCounter.h
    build counter = 0;
    nome app = "AppName"
  Il file BCounter può essere modificato manualmente. Le sei righe
    aggiuntive "\\ comment" possono essere sostituite con righe di
    codice comprensibile al compilatore (es.: #define). Queste saranno
    riportate nel file counter.h (o altro file specificato nel
    BCounter.txt)

  Return code:
  1 = error
  0 = OK
 --------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
	char  FullPath[HRECLEN];
	char  CounterFilePath[128];
	char  HeaderFilePath[128];

	// --------------------------------------------------------------------
	printf("ClevBuild %d.%s\n",CBVER,CBREL);
	if(argc <2 || argc > 2) {
		printf("destination folder not specified\n");
		return(1);
	}

	//memcpy(FullPath,argv[1],strlen(argv[1]));
	strcpy(FullPath,argv[1]);

	// --------------------------------------------------------------------
	// Set default starting point
	// --------------------------------------------------------------------
	for(int i=0;i<BCROWS;i++) {
		memset(DataRows[i],0,HRECLEN);
		if(i>2)
			sprintf(DataRows[i],"//comment %d",i-2);
	}
	sprintf(DataRows[0],"%s","0");						// 0, Counter
	sprintf(DataRows[1],"%s","AppName");				// 1, Nome del programma
	sprintf(DataRows[2],"%s/%s",FullPath,FHEADER);		// 2, header file (output)

	// --------------------------------------------------------------------
	DIR* dir = opendir(FullPath);
	if(dir)
		closedir(dir);
	else if (ENOENT == errno) {
		printf("folder [%s] does not exist\n",FullPath);
		return(1);
	}
	else {
		printf("error accessing folder [%s]\n",FullPath);
		return(1);
	}

	// --------------------------------------------------------------------
	// Header file path = Counter file path
	// --------------------------------------------------------------------
	sprintf(CounterFilePath,"%s/%s",FullPath,FCOUNTER);
	sprintf(HeaderFilePath,"%s/%s",FullPath,FHEADER);

	// --------------------------------------------------------------------
	// 1. Se esiste, apre il counter file. Altrimenti lo crea
	// --------------------------------------------------------------------
	struct stat buffer;
	printf("counter: [%s]\n",CounterFilePath);
	if(!stat(CounterFilePath, &buffer) == 0) {
		if(WriteCounter(CounterFilePath))
			printf("File [%s] does not exist. A new file was created\n",CounterFilePath);
		else
			printf("Error writing [%s]\n",CounterFilePath);
	}
	else {
		if(ReadCounter(CounterFilePath))
			printf("Error reading [%s]\n",CounterFilePath);
		else {
			sprintf(DataRows[0],"%d",++BuildNum);
			strcpy(HeaderFilePath,DataRows[2]);
			printf("header [%s]\n",HeaderFilePath);
			if(WriteHeader(HeaderFilePath))
				printf("Error writing [%s]\n",HeaderFilePath);
			else {
				WriteCounter(CounterFilePath);
				printf("Build version [%d]\n",BuildNum);
			}
		}
	}

	return(0);
}

/*--------------------------------------------------------------------------
--------------------------------------------------------------------------*/
char* getFileNameFromPath(char* path)
{
	for(size_t i = strlen(path) - 1; i; i--)
		if (path[i] == '/')
			return(&path[i+1]);
	return(path);
}

/*--------------------------------------------------------------------------
--------------------------------------------------------------------------*/
char* getParentFromPath(char* path)
{
	for(size_t i = strlen(path) - 1; i; i--) {
		if (path[i] == '/') {
			path[i]='\0';
			break;
		}
	}
	return(path);
}

/*--------------------------------------------------------------------------
  Return code:
  1 = error
  0 = OK
--------------------------------------------------------------------------*/
int WriteCounter(char *fullpath)
{
	FILE *pHc;
	int  Result = 1;

	pHc=fopen(fullpath,"w");
	if(pHc != NULL) {
		for(int i=0;i<BCROWS;i++)
			fprintf(pHc,"%s\n",DataRows[i]);
		fflush(pHc);
		fclose(pHc);
		// printf("write Build number = %s\n",DataRows[0]);
		Result = 0;
	}
	return(Result);
}

/*--------------------------------------------------------------------------
  Return code:
  1 = error
  0 = OK
--------------------------------------------------------------------------*/
int ReadCounter(char *fullpath)
{
	FILE *pHc;
	int  Result = 1;
	int  i=0,L;

	pHc=fopen(fullpath,"r+");
	if(pHc != NULL) {
		while(!feof(pHc) && i < BCROWS) {
			fgets(DataRows[i],HRECLEN,pHc);
			L = strlen(DataRows[i]);
			DataRows[i][L-1]='\0';
			++i;
		}
		fclose(pHc);
		BuildNum = atoi(DataRows[0]);
		//printf("Read Build number = %d\n",BuildNum);
		Result = 0;
	}
	return(Result);
}

/*--------------------------------------------------------------------------
  Return code:
  1 = error
  0 = OK
--------------------------------------------------------------------------*/
int  WriteHeader(char *fullpath)
{
	FILE *pHf;
	int  Result = 1;

	pHf=fopen(fullpath,"w");
	if(pHf != NULL) {
		fprintf(pHf,"/* --------------------------------------------------------------------------------\n");
		fprintf(pHf,"   -----------\n");
		fprintf(pHf,"   This header file wass automatically created by ClevBuild v.%d.%s\n",CBVER,CBREL);
		fprintf(pHf,"   See:\n");
		fprintf(pHf,"    Eclipse:\n");
		fprintf(pHf,"    - Project/Properties/Build Steps/Pre-build steps\n");
		fprintf(pHf,"    and properly set the project configuration\n");
		fprintf(pHf,"   To reset build numbering or set header file comments, edit or reset the file\n");
		fprintf(pHf,"   Comments can be changed into valid source code (e.g. #define or declarations)\n");
		fprintf(pHf,"-------------------------------------------------------------------------------- */\n");
		fprintf(pHf,"#ifndef _CNTBuild\n");
		fprintf(pHf,"#define _CNTBuild\n");
		fprintf(pHf,"#define BUILDNUMBER %d\n",BuildNum);
		fprintf(pHf,"#define BUILDNAME \"%s\"\n",DataRows[1]);
		for(int i=3;i<8;i++)
			fprintf(pHf,"   %s\n",DataRows[i]);
		fprintf(pHf,"#endif\n");
		fclose(pHf);
		Result=0;
	}

	return(Result);
}

