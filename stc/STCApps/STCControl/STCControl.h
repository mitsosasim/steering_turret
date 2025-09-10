/* =======================================================================================================
  STCControl.h
  Created on: Feb 19, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 19, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#ifndef STCCONTROL_H_
#define STCCONTROL_H_

#define MAXLINE 1024

bool CheckData(void);
bool CreateIPC(void);
void UDPClient(void);

void InitLocalData(void);
int  GetIPAddressPort();
void InitWorkTimers(ULONG *,ULONG *);
void UpdateWorkTimers(ULONG,ULONG);
bool FuncFileExist(const char *);

void* ThCANOperator(void *);
void* ThListener(void *);
void* ThTalker(void *);
void* ThSafety(void *);
void* ThSelfLevl(void*);

typedef enum _event {
	RPTSWVER = 1,
	RPTCONFIG,
	RPTERRORS,
	RPTFULL,
	RPTWRKTIME,
	RPTLOG,
	RPTPOSDATA,
}EVENT;
void AddEvent(EVENT);
bool GetEvent(EVENT *);


#endif /* STCCONTROL_H_ */
