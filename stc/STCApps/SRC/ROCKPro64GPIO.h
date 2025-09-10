#ifndef _ROCKPro64GPIO_h_ 
#define _ROCKPro64GPIO_h_
/* 
 * Functions to handle GPIO connector on the ROCKPro64 board.
 * 
 * NOT thread-safe!
 * 
 * stderr used for error handling.
 * 
 * GPIO blocks supported:
 * GPIO1_A
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <errno.h>   /* Error number definitions */
#include <sys/time.h>
#include <time.h>
#include <signal.h>
#include <ctype.h>
#include <termios.h>
#include <sys/mman.h>
#include <stdint.h>   /* for uint64_t */
#include <sched.h>
#include <pthread.h>

/* Shared memory is used for global control of access to certain registers. 
 * Here the relevant structures.
 */
struct GPIOSharedMemoryInfoStruct 
{
  char Name[64];
  int MD;
  off_t Len;
  void *Data;
} GPIOSharedMemoryInfo;
  
#define GPIOSharedMemoryAttachTimeOut 10.0 /* This timeout is in seconds, but it may be a double value.  */ 

/* Structures to control GPIO clock. First process, which modifies clock sets also variable remembering the original state.
 * Further processes, which use clock can subscribe to current setting.
 * Change of clock status may be done only if no processes subscribe to the cusrrent setting.
 * Any changes are protected by mutex.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * Currently it is assumed that clock-on is the only state actively set and used by this library.
 * So any process, using this library, will not request the clock to be off. Also it will not protect
 * the off-state of clock by subscribing to it (increasing the counter variable ProcessesUsingCurrentState when clock is set to off).
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */ 
enum GPIOClockState
{
  GPIOClockStateInvalid = -1,
  GPIOClockStateEnabled = 0UL,
  GPIOClockStateDisabled = 1UL
};

struct GPIOClockAccessControl
{
  pthread_mutex_t Mutex;
  int ProcessesUsingCurrentState;
  enum GPIOClockState OriginalState;
  enum GPIOClockState CurrentState;  /* Current state can be read from the register. This is to check for any mismatch. */
};

struct GPIODirectionAccessControl
{
  pthread_mutex_t Mutex;
};
struct GPIOSetStateAccessControl
{
  pthread_mutex_t Mutex;
};

struct GPIOAccessControlStruct
{
  int Version;
  struct GPIOClockAccessControl GPIOClockAccessControlGPIO1;
  struct GPIOClockAccessControl GPIOClockAccessControlGPIO3;
  struct GPIOClockAccessControl GPIOClockAccessControlGPIO4;
  struct GPIODirectionAccessControl GPIODirectionAccessControlGPIO1;
  struct GPIODirectionAccessControl GPIODirectionAccessControlGPIO3;
  struct GPIODirectionAccessControl GPIODirectionAccessControlGPIO4;
  struct GPIOSetStateAccessControl GPIOSetStateAccessControlGPIO1;
  struct GPIOSetStateAccessControl GPIOSetStateAccessControlGPIO3;
  struct GPIOSetStateAccessControl GPIOSetStateAccessControlGPIO4;

} *GPIOAccessControl;


/* End of entries related to shared memory area.  
 */
 
/* Different pins exposed at the GPIO connector may have different functions.
 * Current function is determined by setting the relevant multiplexer. 
 * The below enum attempts to lists all implemented functions of all implemented pins.
 * This library checks functions available to a given pin, when setting 
 * this functions in the calls GPIOPinModeSetXXXX(). 
 */
enum GPIOPinMode
{
  GPIOPinModeInvalid = 0,/* 0 is an invalid mode. */
  GPIOPinModeGPIO, 
  GPIOPinModeUART4,
  GPIOPinModeSPI1,
  GPIOPinModeI2C4,
  GPIOPinModePMUM0JTAG,
  GPIOPinModeI2C8,
  GPIOPinModeSPI3,
  GPIOPinModeI2S0,
  GPIOPinModeSPDIF,
  GPIOPinModeUART2,
  GPIOPinModeUARTHDCP,
  GPIOPinModePWM      /* This is exclusiverly for GPIO4_C6, which is exposed at J8 connector - Not in the big GPIO connector. */
};

enum GPIOPullUpDownState
{
  GPIOPullUpDownStateNormal = 0,
  GPIOPullUpDownStateUp = 1,
  GPIOPullUpDownStateDown = 2
};
enum GPIODriveStrengthState
{
  GPIODriveStrengthState3mA = 0,
  GPIODriveStrengthState6mA = 1,
  GPIODriveStrengthState9mA = 2,
  GPIODriveStrengthState12mA = 3
};

enum GPIOPinState
{
  GPIOPinState0 = 0,
  GPIOPinState1 = 1
};

enum GPIOPinDirection
{
  GPIOPinDirectionInput = 0,
  GPIOPinDirectionOutput = 1
};

enum GPIOBlock
{
  GPIOBlockGPIO1 = 1,
  GPIOBlockGPIO2,
  GPIOBlockGPIO3,
  GPIOBlockGPIO4
};

enum GPIOSubBlock
{
  GPIOSubBlockA = 1,
  GPIOSubBlockB,
  GPIOSubBlockC,
  GPIOSubBlockD
};

struct GPIOPinHandlingStructure
{
  enum GPIOBlock Block;
  enum GPIOSubBlock SubBlock;
  int Pin;   /* This is the pin number in the GPIO block/sub block.
                Pin = -1 marks not handled or invalid pin. */
  enum GPIOPinMode ModeSet;
  int (*GPIOPinModeSet)( int , enum GPIOPinMode);
  int (*GPIOClockEnable)( );
  int (*GPIOClockReturnToPrevious)( );
  int (*GPIOSetPinDirection)(int, enum GPIOPinDirection);
  int (*GPIOSetPinState)(int, enum GPIOPinState );
  int (*GPIOSetPinState_unlocked)(int, enum GPIOPinState );
  enum GPIOPinState (*GPIOGetPinState)( int );
  int (*GPIOSetPullUpDown)( int , enum GPIOPullUpDownState  );
  int (*GPIODriveStrength)( int , enum GPIODriveStrengthState );
  enum GPIOPullUpDownState (*GPIOGetPullUpDown)( int );
  int (*GPIOLock)();
  int (*GPIOUnlock)();

};

int GPIOAccessControlMutexInit( pthread_mutex_t *);
int GPIOSharedMemoryInfoStructureInit( void);
int GPIOSharedMemoryInit(void);
int GPIOSharedMemoryAttach( void);
int GPIOClockAccessControlInit( struct GPIOClockAccessControl *);
int GPIOInitHWMemoryAccess( void);
int GPIOInit( void);

int GPIOClockEnableGPIO1( void);
int GPIOClockReturnToPreviousGPIO1( void);
int GPIOClockEnableGPIO3( );
int GPIOClockReturnToPreviousGPIO3( );
int GPIOClockEnableGPIO4( );
int GPIOClockReturnToPreviousGPIO4( );

int GPIOPinModeSetGPIO1_A( int , enum GPIOPinMode );
int GPIOPinModeSetGPIO1_B( int , enum GPIOPinMode );
int GPIOPinModeSetGPIO1_C( int , enum GPIOPinMode );
int GPIOPinModeSetGPIO3_D( int , enum GPIOPinMode );
int GPIOPinModeSetGPIO4_C( int , enum GPIOPinMode );
int GPIOPinModeSetGPIO4_D( int , enum GPIOPinMode );

int GPIOSetPinDirectionGPIO1_A( int, enum GPIOPinDirection );
int GPIOSetPinDirectionGPIO1_B( int, enum GPIOPinDirection );
int GPIOSetPinDirectionGPIO1_C( int, enum GPIOPinDirection );
int GPIOSetPinDirectionGPIO3_D( int, enum GPIOPinDirection );
int GPIOSetPinDirectionGPIO4_C( int, enum GPIOPinDirection );
int GPIOSetPinDirectionGPIO4_D( int, enum GPIOPinDirection );

int GPIOSetPinStateGPIO1_A( int, enum GPIOPinState );
int GPIOSetPinStateGPIO1_B( int, enum GPIOPinState );
int GPIOSetPinStateGPIO1_C( int, enum GPIOPinState );
int GPIOSetPinStateGPIO3_D( int, enum GPIOPinState );
int GPIOSetPinStateGPIO4_C( int, enum GPIOPinState );
int GPIOSetPinStateGPIO4_D( int, enum GPIOPinState );

int GPIOSetPinStateGPIO1_A_unlocked( int, enum GPIOPinState );
int GPIOSetPinStateGPIO1_B_unlocked( int, enum GPIOPinState );
int GPIOSetPinStateGPIO1_C_unlocked( int, enum GPIOPinState );
int GPIOSetPinStateGPIO3_D_unlocked( int, enum GPIOPinState );
int GPIOSetPinStateGPIO4_C_unlocked( int, enum GPIOPinState );
int GPIOSetPinStateGPIO4_D_unlocked( int, enum GPIOPinState );

enum GPIOPinState GPIOGetPinStateGPIO1_A( int );
enum GPIOPinState GPIOGetPinStateGPIO1_B( int );
enum GPIOPinState GPIOGetPinStateGPIO1_C( int );
enum GPIOPinState GPIOGetPinStateGPIO3_D( int );
enum GPIOPinState GPIOGetPinStateGPIO4_C( int );
enum GPIOPinState GPIOGetPinStateGPIO4_D( int );

int GPIOSetPullUpDownGPIO1_A( int , enum GPIOPullUpDownState  );
int GPIOSetPullUpDownGPIO1_B( int , enum GPIOPullUpDownState  );
int GPIOSetPullUpDownGPIO1_C( int , enum GPIOPullUpDownState  );
int GPIOSetPullUpDownGPIO3_D( int , enum GPIOPullUpDownState  );
int GPIOSetPullUpDownGPIO4_C( int , enum GPIOPullUpDownState  );
int GPIOSetPullUpDownGPIO4_D( int , enum GPIOPullUpDownState  );

enum GPIOPullUpDownState GPIOGetPullUpDownGPIO1_A( int );
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO1_B( int );
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO1_C( int );
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO3_D( int );
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO4_C( int );
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO4_D( int );

int GPIODriveStrengthGPIO1_A( int , enum GPIODriveStrengthState );
int GPIODriveStrengthGPIO1_B( int , enum GPIODriveStrengthState );
int GPIODriveStrengthGPIO1_C( int , enum GPIODriveStrengthState );
int GPIODriveStrengthGPIO3_D( int , enum GPIODriveStrengthState );
int GPIODriveStrengthGPIO4_C( int , enum GPIODriveStrengthState );
int GPIODriveStrengthGPIO4_D( int , enum GPIODriveStrengthState );

int GPIOLockGPIO1( );
int GPIOUnlockGPIO1( );
int GPIOLockGPIO3( );
int GPIOUnlockGPIO3( );
int GPIOLockGPIO4( );
int GPIOUnlockGPIO4( );


int GPIOPinInit( int, enum GPIOPinMode);
int GPIOPinRelease( int );
int GPIOSetPinDirection( int , enum GPIOPinDirection );
int GPIOSetPinState( int , enum GPIOPinState  );
int GPIOSetPinState_unlocked( int , enum GPIOPinState  );
enum GPIOPinState GPIOGetPinState( int  );
int GPIOSetPullUpDown( int , enum GPIOPullUpDownState  );
int GPIODriveStrength( int , enum GPIODriveStrengthState  );
enum GPIOPullUpDownState GPIOGetPullUpDown( int );

#endif /* _ROCKPro64GPIO_h_ */
