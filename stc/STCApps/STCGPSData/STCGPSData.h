/* =======================================================================================================
   main.h

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
======================================================================================================= */
#ifndef MAIN_H_
#define MAIN_H_

#define MAXNMEALEN 256

int  ReadGPSData();
int  ReadGPSFile();
int  CheckRSParams();
int  SetRSPort();
int  CheckCRC(char *);
int  NMEARSRead(char *);
void NMEAParse(char *);
void InitLocalData();

#endif /* MAIN_H_ */
