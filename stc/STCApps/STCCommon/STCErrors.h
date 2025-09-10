/* =======================================================================================================
  STCErrCodes.h

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
      R. Filippone
      A. Kamal Eddine
 ------------------------------------------------------------------------------


======================================================================================================= */
#ifndef ERRCODES_H_
#define ERRCODES_H_

#include "STCTypes.h"

#define STCNOERROR    0
#define CFGERR131 // - record too long
#define CFGERR132 // - record too short
#define CFGERR133 // - label or value wrong size
#define CFGERR134 // - wrong syntax
#define CFGERR135 // - unknown parameter
#define CFGERR136 // - unknown section

#define NETERROR	400	// Network error
#define PROTERR		401	// Protocol error
#define MEMERROR	402 // Memory error
#define IPCERROR	403 // IPC error
#define FILERROR	404 // File error
#define RSWRITE		405 // Error writing on UART

void STCErrorLog(char*,LOGLEV,STCAPPID,UINT,char*);

#endif /* ERRCODES_H_ */

