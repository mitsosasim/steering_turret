#ifndef _ROCKPro64GPIO_Globals_h_ 
#define _ROCKPro64GPIO_Globals_h_

#include "ROCKPro64GPIO.h"

extern int GPIOAccessControlVersion;  /* This number should increase, if changes are done in shared structures, and released! */

/* Memory areas, which will be mapped to hardware.  */
extern void *PMUCRU_base;
extern void *PMUGRF_base;
extern void *CRU_base;
extern void *GRF_base;
extern void *PMU_base;
extern void *GPIO1_base;
extern void *GPIO3_base;
extern void *GPIO4_base;

/* Mapping of pins in the big GPIO connector to RK3399 GPIO's is maintained in this array.
 * It is used by the high-level functions, which accept pin 
 * numbering of the GPIO connector on ROCKPro64 board.
 * 
 * The pins have numbers from 1 to 40. Not very C-like...
 * Pin 0 will be used artificially for the PWM line, which is goes to connector J8 
 * on the ROCKPro64 board.
 */
 
extern struct GPIOPinHandlingStructure GPIOPinMap[ 42];


/* These definitions are for issuing error messages. Not really related to program flow.
 * I want to have then consistently in all files, but not necesarily in the function prototypes.
 */
#define WarningMessage( ErrMess) fprintf(stderr, "%s() (file %s, line %d): Warning: %s\n", \
    __func__, __FILE__, __LINE__, (ErrMess) )
#define ErrorMessage( ErrMess) fprintf(stderr, "%s() (file %s, line %d): Error: %s\n", \
    __func__, __FILE__, __LINE__, (ErrMess) )
#define ErrorMessageFromCall(Func) fprintf(stderr, "%s() (file %s, line %d): Error from function %s(): Errno: (%d) [%s]\n", \
    __func__, __FILE__, __LINE__, (Func), errno, strerror(errno))
#define ErrorMessageFromCall1(Func) fprintf(stderr, "%s() (file %s, line %d): Error from function %s()\n", \
    __func__, __FILE__, __LINE__, (Func))

#endif /* _ROCKPro64GPIO_Globals_h_ */
