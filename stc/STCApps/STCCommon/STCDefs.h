/* =======================================================================================================
  STCDefs.h

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
 ------------------------------------------------------------------------------


======================================================================================================= */
#ifndef STCDEFS_H_
#define STCDEFS_H_

#define MAXPATHLEN 128
#define CFGPATH  "./config"
#define CFGFILE  "STCDefault.cfg"
#define LOGPATH  "./log"
#define LOGFILE  "STCctrl_"
#define DATAPATH "./data"
#define TIMEFILE "STCtime.dat"

#define FALSE  0
#define TRUE   1

#define SYSDATASHMKEY 2001		// System Data
#define SYSDATASEMKEY 3001		// System Data access control

#define LOGDATAQUEKEY 4001		// Queue for log msgs
#define SLCMDQUEKEY   4002		// Queue for SL cmds
#define CANCMDQUEKEY  4003		// Queue for CAN cmds

#define STCPROTOCOL 	 "1.0a"
#define MAX_USERS		 6

#endif /* STCDEFS_H_ */


