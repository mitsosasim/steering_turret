/* 
 * Functions to handle GPIO connector on the ROCKPro64 board.
 * 
 * Some functions are NOT thread-safe! 
 * 
 * stderr used for error handling.
 * 
 */


#include "ROCKPro64GPIO.h"
#include "ROCKPro64GPIO_Globals.h"

/* Variables related to shared controls.
 */
int GPIOAccessControlVersion= 1;  /* This number should increase, 
                        if changes are done in shared structures, 
                        or their handling, and released! This is aimed at
                        avoiding concurrent usage of incompatible versions
                        of this library. */

/* End of entries related to shared memory area.  
 */

/* Memory areas, which will be mapped to hardware.  */
void *PMUCRU_base= NULL;
void *PMUGRF_base= NULL;
void *CRU_base= NULL;
void *GRF_base= NULL;
void *PMU_base= NULL;
void *GPIO1_base= NULL;
void *GPIO3_base= NULL;
void *GPIO4_base= NULL;

/* Mapping of pins in the big GPIO connector to RK3399 GPIO's is maintained in this array.
 * It is used by the high-level functions, which accept pin 
 * numbering of the GPIO connector on ROCKPro64 board.
 * 
 * The pins have numbers from 1 to 40. Not very C-like...
 * Pin 0 will be used artificially for the PWM line, which is goes to connector J8 
 * on the ROCKPro64 board.
 */
 
struct GPIOPinHandlingStructure GPIOPinMap[ 42];

#define MAP_64k (64*1024UL)
#define MAP_64k_MASK (MAP_64k - 1)
#define MAP_32k (32*1024UL)
#define MAP_32k_MASK (MAP_32k - 1)


int GPIOInitPinMap()
{
  int i;
  
/*
 *   GPIOPinMap[] is a global variable, so all its bytes are initialized to 0
 * by standard.
 * We just need to make sure that Pin is -1 at all places which are not implemented.
 */
  for(i= 0; i<41; i++)
  {
    GPIOPinMap[i].Pin= -1;
  }
/* Pins in the GPIO connector are numbered from 1 through 40.
 * Position 0 in this library is used artificially for the PWM channel,
 * which is connected to J8 connector on the ROCKPro64 board.
 */  
/*!!!ToDo:
 * Check:
 *  1. If GPIO4_C6 is really exposed at J8 connector.
 *  2. What voltage is has.
 *  3. Is it somehow hardwired or default to PWM.
 */
  GPIOPinMap[0].Block= GPIOBlockGPIO4;
  GPIOPinMap[0].SubBlock= GPIOSubBlockC;
  GPIOPinMap[0].Pin= 6;
  GPIOPinMap[0].GPIOPinModeSet= GPIOPinModeSetGPIO4_C;
  GPIOPinMap[0].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[0].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[0].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_C;
  GPIOPinMap[0].GPIOSetPinState= GPIOSetPinStateGPIO4_C;
  GPIOPinMap[0].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_C_unlocked;
  GPIOPinMap[0].GPIOGetPinState= GPIOGetPinStateGPIO4_C;
  GPIOPinMap[0].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_C;
  GPIOPinMap[0].GPIODriveStrength= GPIODriveStrengthGPIO4_C;
  GPIOPinMap[0].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_C;
  GPIOPinMap[0].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[0].GPIOUnlock= GPIOUnlockGPIO4;
  
/* Now the 'regular' GPIO pins: */  
  GPIOPinMap[3].Block= GPIOBlockGPIO1;
  GPIOPinMap[3].SubBlock= GPIOSubBlockC;
  GPIOPinMap[3].Pin= 4;
  GPIOPinMap[3].GPIOPinModeSet= GPIOPinModeSetGPIO1_C;
  GPIOPinMap[3].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[3].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[3].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_C;
  GPIOPinMap[3].GPIOSetPinState= GPIOSetPinStateGPIO1_C;
  GPIOPinMap[3].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_C_unlocked;
  GPIOPinMap[3].GPIOGetPinState= GPIOGetPinStateGPIO1_C;
  GPIOPinMap[3].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_C;
  GPIOPinMap[3].GPIODriveStrength= GPIODriveStrengthGPIO1_C;
  GPIOPinMap[3].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_C;
  GPIOPinMap[3].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[3].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[5].Block= GPIOBlockGPIO1;
  GPIOPinMap[5].SubBlock= GPIOSubBlockC;
  GPIOPinMap[5].Pin= 5;
  GPIOPinMap[5].GPIOPinModeSet= GPIOPinModeSetGPIO1_C;
  GPIOPinMap[5].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[5].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[5].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_C;
  GPIOPinMap[5].GPIOSetPinState= GPIOSetPinStateGPIO1_C;
  GPIOPinMap[5].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_C_unlocked;
  GPIOPinMap[5].GPIOGetPinState= GPIOGetPinStateGPIO1_C;
  GPIOPinMap[5].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_C;
  GPIOPinMap[5].GPIODriveStrength= GPIODriveStrengthGPIO1_C;
  GPIOPinMap[5].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_C;
  GPIOPinMap[5].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[5].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[7].Block= GPIOBlockGPIO4;
  GPIOPinMap[7].SubBlock= GPIOSubBlockD;
  GPIOPinMap[7].Pin= 0;
  GPIOPinMap[7].GPIOPinModeSet= GPIOPinModeSetGPIO4_D;
  GPIOPinMap[7].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[7].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[7].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_D;
  GPIOPinMap[7].GPIOSetPinState= GPIOSetPinStateGPIO4_D;
  GPIOPinMap[7].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_D_unlocked;
  GPIOPinMap[7].GPIOGetPinState= GPIOGetPinStateGPIO4_D;
  GPIOPinMap[7].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_D;
  GPIOPinMap[7].GPIODriveStrength= GPIODriveStrengthGPIO4_D;
  GPIOPinMap[7].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_D;
  GPIOPinMap[7].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[7].GPIOUnlock= GPIOUnlockGPIO4;

  GPIOPinMap[8].Block= GPIOBlockGPIO4;
  GPIOPinMap[8].SubBlock= GPIOSubBlockC;
  GPIOPinMap[8].Pin= 4;
  GPIOPinMap[8].GPIOPinModeSet= GPIOPinModeSetGPIO4_C;
  GPIOPinMap[8].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[8].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[8].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_C;
  GPIOPinMap[8].GPIOSetPinState= GPIOSetPinStateGPIO4_C;
  GPIOPinMap[8].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_C_unlocked;
  GPIOPinMap[8].GPIOGetPinState= GPIOGetPinStateGPIO4_C;
  GPIOPinMap[8].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_C;
  GPIOPinMap[8].GPIODriveStrength= GPIODriveStrengthGPIO4_C;
  GPIOPinMap[8].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_C;
  GPIOPinMap[8].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[8].GPIOUnlock= GPIOUnlockGPIO4;
   
  GPIOPinMap[10].Block= GPIOBlockGPIO4;
  GPIOPinMap[10].SubBlock= GPIOSubBlockC;
  GPIOPinMap[10].Pin= 3;
  GPIOPinMap[10].GPIOPinModeSet= GPIOPinModeSetGPIO4_C;
  GPIOPinMap[10].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[10].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[10].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_C;
  GPIOPinMap[10].GPIOSetPinState= GPIOSetPinStateGPIO4_C;
  GPIOPinMap[10].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_C_unlocked;
  GPIOPinMap[10].GPIOGetPinState= GPIOGetPinStateGPIO4_C;
  GPIOPinMap[10].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_C;
  GPIOPinMap[10].GPIODriveStrength= GPIODriveStrengthGPIO4_C;
  GPIOPinMap[10].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_C;
  GPIOPinMap[10].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[10].GPIOUnlock= GPIOUnlockGPIO4;

  GPIOPinMap[11].Block= GPIOBlockGPIO1;
  GPIOPinMap[11].SubBlock= GPIOSubBlockC;
  GPIOPinMap[11].Pin= 6;
  GPIOPinMap[11].GPIOPinModeSet= GPIOPinModeSetGPIO1_C;
  GPIOPinMap[11].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[11].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[11].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_C;
  GPIOPinMap[11].GPIOSetPinState= GPIOSetPinStateGPIO1_C;
  GPIOPinMap[11].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_C_unlocked;
  GPIOPinMap[11].GPIOGetPinState= GPIOGetPinStateGPIO1_C;
  GPIOPinMap[11].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_C;
  GPIOPinMap[11].GPIODriveStrength= GPIODriveStrengthGPIO1_C;
  GPIOPinMap[11].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_C;
  GPIOPinMap[11].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[11].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[12].Block= GPIOBlockGPIO3;
  GPIOPinMap[12].SubBlock= GPIOSubBlockD;
  GPIOPinMap[12].Pin= 0;
  GPIOPinMap[12].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[12].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[12].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[12].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[12].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[12].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[12].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[12].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[12].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[12].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[12].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[12].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[13].Block= GPIOBlockGPIO1;
  GPIOPinMap[13].SubBlock= GPIOSubBlockC;
  GPIOPinMap[13].Pin= 2;
  GPIOPinMap[13].GPIOPinModeSet= GPIOPinModeSetGPIO1_C;
  GPIOPinMap[13].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[13].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[13].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_C;
  GPIOPinMap[13].GPIOSetPinState= GPIOSetPinStateGPIO1_C;
  GPIOPinMap[13].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_C_unlocked;
  GPIOPinMap[13].GPIOGetPinState= GPIOGetPinStateGPIO1_C;
  GPIOPinMap[13].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_C;
  GPIOPinMap[13].GPIODriveStrength= GPIODriveStrengthGPIO1_C;
  GPIOPinMap[13].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_C;
  GPIOPinMap[13].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[13].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[15].Block= GPIOBlockGPIO1;
  GPIOPinMap[15].SubBlock= GPIOSubBlockA;
  GPIOPinMap[15].Pin= 1;
  GPIOPinMap[15].GPIOPinModeSet= GPIOPinModeSetGPIO1_A;
  GPIOPinMap[15].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[15].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[15].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_A;
  GPIOPinMap[15].GPIOSetPinState= GPIOSetPinStateGPIO1_A;
  GPIOPinMap[15].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_A_unlocked;
  GPIOPinMap[15].GPIOGetPinState= GPIOGetPinStateGPIO1_A;
  GPIOPinMap[15].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_A;
  GPIOPinMap[15].GPIODriveStrength= GPIODriveStrengthGPIO1_A;
  GPIOPinMap[15].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_A;
  GPIOPinMap[15].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[15].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[16].Block= GPIOBlockGPIO1;
  GPIOPinMap[16].SubBlock= GPIOSubBlockA;
  GPIOPinMap[16].Pin= 4;
  GPIOPinMap[16].GPIOPinModeSet= GPIOPinModeSetGPIO1_A;
  GPIOPinMap[16].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[16].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[16].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_A;
  GPIOPinMap[16].GPIOSetPinState= GPIOSetPinStateGPIO1_A;
  GPIOPinMap[16].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_A_unlocked;
  GPIOPinMap[16].GPIOGetPinState= GPIOGetPinStateGPIO1_A;
  GPIOPinMap[16].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_A;
  GPIOPinMap[16].GPIODriveStrength= GPIODriveStrengthGPIO1_A;
  GPIOPinMap[16].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_A;
  GPIOPinMap[16].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[16].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[18].Block= GPIOBlockGPIO4;
  GPIOPinMap[18].SubBlock= GPIOSubBlockC;
  GPIOPinMap[18].Pin= 5;
  GPIOPinMap[18].GPIOPinModeSet= GPIOPinModeSetGPIO4_C;
  GPIOPinMap[18].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[18].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[18].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_C;
  GPIOPinMap[18].GPIOSetPinState= GPIOSetPinStateGPIO4_C;
  GPIOPinMap[18].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_C_unlocked;
  GPIOPinMap[18].GPIOGetPinState= GPIOGetPinStateGPIO4_C;
  GPIOPinMap[18].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_C;
  GPIOPinMap[18].GPIODriveStrength= GPIODriveStrengthGPIO4_C;
  GPIOPinMap[18].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_C;
  GPIOPinMap[18].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[18].GPIOUnlock= GPIOUnlockGPIO4;

  GPIOPinMap[19].Block= GPIOBlockGPIO1;
  GPIOPinMap[19].SubBlock= GPIOSubBlockB;
  GPIOPinMap[19].Pin= 0;
  GPIOPinMap[19].GPIOPinModeSet= GPIOPinModeSetGPIO1_B;
  GPIOPinMap[19].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[19].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[19].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_B;
  GPIOPinMap[19].GPIOSetPinState= GPIOSetPinStateGPIO1_B;
  GPIOPinMap[19].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_B_unlocked;
  GPIOPinMap[19].GPIOGetPinState= GPIOGetPinStateGPIO1_B;
  GPIOPinMap[19].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_B;
  GPIOPinMap[19].GPIODriveStrength= GPIODriveStrengthGPIO1_B;
  GPIOPinMap[19].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_B;
  GPIOPinMap[19].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[19].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[21].Block= GPIOBlockGPIO1;
  GPIOPinMap[21].SubBlock= GPIOSubBlockA;
  GPIOPinMap[21].Pin= 7;
  GPIOPinMap[21].GPIOPinModeSet= GPIOPinModeSetGPIO1_A;
  GPIOPinMap[21].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[21].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[21].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_A;
  GPIOPinMap[21].GPIOSetPinState= GPIOSetPinStateGPIO1_A;
  GPIOPinMap[21].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_A_unlocked;
  GPIOPinMap[21].GPIOGetPinState= GPIOGetPinStateGPIO1_A;
  GPIOPinMap[21].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_A;
  GPIOPinMap[21].GPIODriveStrength= GPIODriveStrengthGPIO1_A;
  GPIOPinMap[21].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_A;
  GPIOPinMap[21].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[21].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[22].Block= GPIOBlockGPIO4;
  GPIOPinMap[22].SubBlock= GPIOSubBlockD;
  GPIOPinMap[22].Pin= 1;
  GPIOPinMap[22].GPIOPinModeSet= GPIOPinModeSetGPIO4_D;
  GPIOPinMap[22].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[22].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[22].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_D;
  GPIOPinMap[22].GPIOSetPinState= GPIOSetPinStateGPIO4_D;
  GPIOPinMap[22].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_D_unlocked;
  GPIOPinMap[22].GPIOGetPinState= GPIOGetPinStateGPIO4_D;
  GPIOPinMap[22].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_D;
  GPIOPinMap[22].GPIODriveStrength= GPIODriveStrengthGPIO4_D;
  GPIOPinMap[22].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_D;
  GPIOPinMap[22].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[22].GPIOUnlock= GPIOUnlockGPIO4;
  
  GPIOPinMap[23].Block= GPIOBlockGPIO1;
  GPIOPinMap[23].SubBlock= GPIOSubBlockB;
  GPIOPinMap[23].Pin= 1;
  GPIOPinMap[23].GPIOPinModeSet= GPIOPinModeSetGPIO1_B;
  GPIOPinMap[23].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[23].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[23].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_B;
  GPIOPinMap[23].GPIOSetPinState= GPIOSetPinStateGPIO1_B;
  GPIOPinMap[23].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_B_unlocked;
  GPIOPinMap[23].GPIOGetPinState= GPIOGetPinStateGPIO1_B;
  GPIOPinMap[23].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_B;
  GPIOPinMap[23].GPIODriveStrength= GPIODriveStrengthGPIO1_B;
  GPIOPinMap[23].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_B;
  GPIOPinMap[23].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[23].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[24].Block= GPIOBlockGPIO1;
  GPIOPinMap[24].SubBlock= GPIOSubBlockB;
  GPIOPinMap[24].Pin= 2;
  GPIOPinMap[24].GPIOPinModeSet= GPIOPinModeSetGPIO1_B;
  GPIOPinMap[24].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[24].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[24].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_B;
  GPIOPinMap[24].GPIOSetPinState= GPIOSetPinStateGPIO1_B;
  GPIOPinMap[24].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_B_unlocked;
  GPIOPinMap[24].GPIOGetPinState= GPIOGetPinStateGPIO1_B;
  GPIOPinMap[24].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_B;
  GPIOPinMap[24].GPIODriveStrength= GPIODriveStrengthGPIO1_B;
  GPIOPinMap[24].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_B;
  GPIOPinMap[24].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[24].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[26].Block= GPIOBlockGPIO1;
  GPIOPinMap[26].SubBlock= GPIOSubBlockB;
  GPIOPinMap[26].Pin= 5;
  GPIOPinMap[26].GPIOPinModeSet= GPIOPinModeSetGPIO1_B;
  GPIOPinMap[26].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[26].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[26].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_B;
  GPIOPinMap[26].GPIOSetPinState= GPIOSetPinStateGPIO1_B;
  GPIOPinMap[26].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_B_unlocked;
  GPIOPinMap[26].GPIOGetPinState= GPIOGetPinStateGPIO1_B;
  GPIOPinMap[26].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_B;
  GPIOPinMap[26].GPIODriveStrength= GPIODriveStrengthGPIO1_B;
  GPIOPinMap[26].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_B;
  GPIOPinMap[26].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[26].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[27].Block= GPIOBlockGPIO1;
  GPIOPinMap[27].SubBlock= GPIOSubBlockB;
  GPIOPinMap[27].Pin= 3;
  GPIOPinMap[27].GPIOPinModeSet= GPIOPinModeSetGPIO1_B;
  GPIOPinMap[27].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[27].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[27].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_B;
  GPIOPinMap[27].GPIOSetPinState= GPIOSetPinStateGPIO1_B;
  GPIOPinMap[27].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_B_unlocked;
  GPIOPinMap[27].GPIOGetPinState= GPIOGetPinStateGPIO1_B;
  GPIOPinMap[27].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_B;
  GPIOPinMap[27].GPIODriveStrength= GPIODriveStrengthGPIO1_B;
  GPIOPinMap[27].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_B;
  GPIOPinMap[27].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[27].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[28].Block= GPIOBlockGPIO1;
  GPIOPinMap[28].SubBlock= GPIOSubBlockB;
  GPIOPinMap[28].Pin= 4;
  GPIOPinMap[28].GPIOPinModeSet= GPIOPinModeSetGPIO1_B;
  GPIOPinMap[28].GPIOClockEnable= GPIOClockEnableGPIO1;
  GPIOPinMap[28].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO1;
  GPIOPinMap[28].GPIOSetPinDirection= GPIOSetPinDirectionGPIO1_B;
  GPIOPinMap[28].GPIOSetPinState= GPIOSetPinStateGPIO1_B;
  GPIOPinMap[28].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO1_B_unlocked;
  GPIOPinMap[28].GPIOGetPinState= GPIOGetPinStateGPIO1_B;
  GPIOPinMap[28].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO1_B;
  GPIOPinMap[28].GPIODriveStrength= GPIODriveStrengthGPIO1_B;
  GPIOPinMap[28].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO1_B;
  GPIOPinMap[28].GPIOLock= GPIOLockGPIO1;
  GPIOPinMap[28].GPIOUnlock= GPIOUnlockGPIO1;

  GPIOPinMap[29].Block= GPIOBlockGPIO4;
  GPIOPinMap[29].SubBlock= GPIOSubBlockD;
  GPIOPinMap[29].Pin= 3;
  GPIOPinMap[29].GPIOPinModeSet= GPIOPinModeSetGPIO4_D;
  GPIOPinMap[29].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[29].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[29].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_D;
  GPIOPinMap[29].GPIOSetPinState= GPIOSetPinStateGPIO4_D;
  GPIOPinMap[29].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_D_unlocked;
  GPIOPinMap[29].GPIOGetPinState= GPIOGetPinStateGPIO4_D;
  GPIOPinMap[29].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_D;
  GPIOPinMap[29].GPIODriveStrength= GPIODriveStrengthGPIO4_D;
  GPIOPinMap[29].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_D;
  GPIOPinMap[29].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[29].GPIOUnlock= GPIOUnlockGPIO4;

  GPIOPinMap[31].Block= GPIOBlockGPIO4;
  GPIOPinMap[31].SubBlock= GPIOSubBlockD;
  GPIOPinMap[31].Pin= 4;
  GPIOPinMap[31].GPIOPinModeSet= GPIOPinModeSetGPIO4_D;
  GPIOPinMap[31].GPIOClockEnable= GPIOClockEnableGPIO4;
  GPIOPinMap[31].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO4;
  GPIOPinMap[31].GPIOSetPinDirection= GPIOSetPinDirectionGPIO4_D;
  GPIOPinMap[31].GPIOSetPinState= GPIOSetPinStateGPIO4_D;
  GPIOPinMap[31].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO4_D_unlocked;
  GPIOPinMap[31].GPIOGetPinState= GPIOGetPinStateGPIO4_D;
  GPIOPinMap[31].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO4_D;
  GPIOPinMap[31].GPIODriveStrength= GPIODriveStrengthGPIO4_D;
  GPIOPinMap[31].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO4_D;
  GPIOPinMap[31].GPIOLock= GPIOLockGPIO4;
  GPIOPinMap[31].GPIOUnlock= GPIOUnlockGPIO4;
  
  GPIOPinMap[32].Block= GPIOBlockGPIO3;
  GPIOPinMap[32].SubBlock= GPIOSubBlockD;
  GPIOPinMap[32].Pin= 4;
  GPIOPinMap[32].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[32].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[32].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[32].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[32].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[32].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[32].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[32].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[32].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[32].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[32].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[32].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[33].Block= GPIOBlockGPIO3;
  GPIOPinMap[33].SubBlock= GPIOSubBlockD;
  GPIOPinMap[33].Pin= 5;
  GPIOPinMap[33].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[33].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[33].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[33].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[33].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[33].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[33].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[33].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[33].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[33].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[33].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[33].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[35].Block= GPIOBlockGPIO3;
  GPIOPinMap[35].SubBlock= GPIOSubBlockD;
  GPIOPinMap[35].Pin= 2;
  GPIOPinMap[35].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[35].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[35].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[35].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[35].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[35].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[35].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[35].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[35].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[35].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[35].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[35].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[36].Block= GPIOBlockGPIO3;
  GPIOPinMap[36].SubBlock= GPIOSubBlockD;
  GPIOPinMap[36].Pin= 6;
  GPIOPinMap[36].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[36].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[36].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[36].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[36].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[36].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[36].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[36].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[36].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[36].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[36].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[36].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[37].Block= GPIOBlockGPIO3;
  GPIOPinMap[37].SubBlock= GPIOSubBlockD;
  GPIOPinMap[37].Pin= 1;
  GPIOPinMap[37].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[37].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[37].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[37].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[37].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[37].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[37].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[37].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[37].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[37].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[37].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[37].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[38].Block= GPIOBlockGPIO3;
  GPIOPinMap[38].SubBlock= GPIOSubBlockD;
  GPIOPinMap[38].Pin= 3;
  GPIOPinMap[38].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[38].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[38].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[38].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[38].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[38].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[38].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[38].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[38].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[38].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[38].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[38].GPIOUnlock= GPIOUnlockGPIO3;

  GPIOPinMap[40].Block= GPIOBlockGPIO3;
  GPIOPinMap[40].SubBlock= GPIOSubBlockD;
  GPIOPinMap[40].Pin= 7;
  GPIOPinMap[40].GPIOPinModeSet= GPIOPinModeSetGPIO3_D;
  GPIOPinMap[40].GPIOClockEnable= GPIOClockEnableGPIO3;
  GPIOPinMap[40].GPIOClockReturnToPrevious= GPIOClockReturnToPreviousGPIO3;
  GPIOPinMap[40].GPIOSetPinDirection= GPIOSetPinDirectionGPIO3_D;
  GPIOPinMap[40].GPIOSetPinState= GPIOSetPinStateGPIO3_D;
  GPIOPinMap[40].GPIOSetPinState_unlocked= GPIOSetPinStateGPIO3_D_unlocked;
  GPIOPinMap[40].GPIOGetPinState= GPIOGetPinStateGPIO3_D;
  GPIOPinMap[40].GPIOSetPullUpDown= GPIOSetPullUpDownGPIO3_D;
  GPIOPinMap[40].GPIODriveStrength= GPIODriveStrengthGPIO3_D;
  GPIOPinMap[40].GPIOGetPullUpDown= GPIOGetPullUpDownGPIO3_D;
  GPIOPinMap[40].GPIOLock= GPIOLockGPIO3;
  GPIOPinMap[40].GPIOUnlock= GPIOUnlockGPIO3;

  return 0;
}

int GPIOSharedMemoryInfoStructureInit()
{
  strcpy( GPIOSharedMemoryInfo.Name, "/GPIOSharedMemory");
  GPIOSharedMemoryInfo.Len= sizeof( struct GPIOAccessControlStruct);
  GPIOSharedMemoryInfo.MD= -100;
  GPIOSharedMemoryInfo.Data= NULL;
  return 0;
}

/* Several access control structures consist just of a mutex.
 * In oreder to speed-up their initialization, just mutex is initialized.
 */
int GPIOAccessControlMutexInit( pthread_mutex_t *Mut)
{
  int i;
  pthread_mutexattr_t attr;

  i= pthread_mutexattr_init(&attr); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutexattr_init(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutexattr_setpshared(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i= pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutexattr_setrobust(). The function returned: %d\n",
            __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i = pthread_mutex_init( Mut, &attr); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_init(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i = pthread_mutexattr_destroy(&attr); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Warning: Error from pthread_mutexattr_destroy(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
     /*return 1;  This might not be so critical. So no return at this stage. */
  }
  
  return 0;
}

/* Initialisation of control structure of GPIO clock in shared memory.  */
int GPIOClockAccessControlInit( struct GPIOClockAccessControl *GPIOCAC)
{
  int i;
  pthread_mutexattr_t attr;

  i= pthread_mutexattr_init(&attr); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutexattr_init(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutexattr_setpshared(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i= pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutexattr_setrobust(). The function returned: %d\n",
            __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i = pthread_mutex_init(&( GPIOCAC->Mutex), &attr); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_init(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -1;
  }
  i = pthread_mutexattr_destroy(&attr); 
  if( i)   
  {
    fprintf( stderr, "%s() (file %s, line %d): Warning: Error from pthread_mutexattr_destroy(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
     /*return 1;  This might not be so critical. So no return at this stage. */
  }
  
  GPIOCAC->ProcessesUsingCurrentState= 0;  /* Well, it is initialisation, so no process could have been using this clock, so far.  */
  GPIOCAC->OriginalState= GPIOClockStateInvalid; /* These fields will be set properly at the first attempt to handle clock.  */
  GPIOCAC->CurrentState= GPIOClockStateInvalid;
  
  return 0;
}

int GPIOSharedMemoryInit()
{
  mode_t Mod;
  int i;

  if( GPIOSharedMemoryInfo.MD != -100)  /* The info structure has not been initialised!  */
  {
    GPIOSharedMemoryInfoStructureInit();
  }
  Mod= 0;  /* In case some higher bits of mode should be set, do it here.  */
  Mod &= ~(S_IRWXU | S_IRWXG | S_IRWXO | S_ISUID | S_ISGID | S_ISVTX); /* If shared memory is opened here, 
                                                          it will not be accessible for other processes,
                                                          until it is initialized completely.  */
  GPIOSharedMemoryInfo.MD= shm_open( GPIOSharedMemoryInfo.Name, O_RDWR | O_CREAT | O_EXCL, Mod);
  if( GPIOSharedMemoryInfo.MD == -1)
  {
    if( errno == EEXIST ) return 1;
    ErrorMessageFromCall("shm_open");
    return -1;
  }
  i= ftruncate( GPIOSharedMemoryInfo.MD, GPIOSharedMemoryInfo.Len);
  if( i)
  {
    ErrorMessageFromCall("ftruncate");
    return -2;
  }
  GPIOSharedMemoryInfo.Data= mmap(NULL, GPIOSharedMemoryInfo.Len, PROT_READ | PROT_WRITE, MAP_SHARED, 
                         GPIOSharedMemoryInfo.MD, 0 ); 
  if( GPIOSharedMemoryInfo.Data == MAP_FAILED)
  {
    ErrorMessageFromCall("mmap");
    return -3;
  }
  GPIOAccessControl= (struct GPIOAccessControlStruct *)GPIOSharedMemoryInfo.Data;
  GPIOAccessControl->Version= GPIOAccessControlVersion;
/*!!!ToDo:
 * Check return value from the below functions!
 */
  i= GPIOClockAccessControlInit( &(GPIOAccessControl->GPIOClockAccessControlGPIO1));
  i= GPIOClockAccessControlInit( &(GPIOAccessControl->GPIOClockAccessControlGPIO3));
  i= GPIOClockAccessControlInit( &(GPIOAccessControl->GPIOClockAccessControlGPIO4));
  
  i= GPIOAccessControlMutexInit( &(GPIOAccessControl->GPIODirectionAccessControlGPIO1.Mutex));
  i= GPIOAccessControlMutexInit( &(GPIOAccessControl->GPIODirectionAccessControlGPIO3.Mutex));
  i= GPIOAccessControlMutexInit( &(GPIOAccessControl->GPIODirectionAccessControlGPIO4.Mutex));

  i= GPIOAccessControlMutexInit( &(GPIOAccessControl->GPIOSetStateAccessControlGPIO1.Mutex));
  i= GPIOAccessControlMutexInit( &(GPIOAccessControl->GPIOSetStateAccessControlGPIO3.Mutex));
  i= GPIOAccessControlMutexInit( &(GPIOAccessControl->GPIOSetStateAccessControlGPIO4.Mutex));

/* Release the shared memory for other processes. */
  Mod = (S_IRWXU | S_IRWXG | S_IRWXO );
  i= fchmod(GPIOSharedMemoryInfo.MD, Mod);
  if( i)
  {
    ErrorMessageFromCall("fchmod");
    return -4;
  }
  return 0;
  
}

int GPIOSharedMemoryAttach()
{
  mode_t Mod;
  struct timespec T0;
  struct timespec T1;
  double TimeElapsed;
  
  if( GPIOSharedMemoryInfo.MD != -100)  /* The info structure has not been initialised!  */
  {
    GPIOSharedMemoryInfoStructureInit();
  }
  
  Mod= 0;
  Mod &= (S_IRWXU | S_IRWXG | S_IRWXO );

  clock_gettime(CLOCK_MONOTONIC, &T0);
  TimeElapsed= 0.;
  
  while( TimeElapsed < GPIOSharedMemoryAttachTimeOut )
  {
    clock_gettime(CLOCK_MONOTONIC, &T1);
    TimeElapsed= ((double)(T1.tv_sec)+ ((double)(T1.tv_nsec)/1000000000.)
                - (double)(T0.tv_sec)- ((double)(T0.tv_nsec)/1000000000.));
    GPIOSharedMemoryInfo.MD= shm_open( GPIOSharedMemoryInfo.Name, O_RDWR , Mod);
    if( GPIOSharedMemoryInfo.MD == -1)
    {
      if( errno == EACCES )
      {
        sched_yield();
        continue; /* Wait until shared memory gets initialized. */
      }
      ErrorMessageFromCall("shm_open");
      return -1;
    }
    break;
  }
  if( GPIOSharedMemoryInfo.MD == -1)
  {
    /* This must have been timeout. */
    ErrorMessageFromCall("shm_open");
    return -2;
  }
  GPIOSharedMemoryInfo.Data= mmap(NULL, GPIOSharedMemoryInfo.Len, PROT_READ | PROT_WRITE, MAP_SHARED, 
                         GPIOSharedMemoryInfo.MD, 0 ); 
  if( GPIOSharedMemoryInfo.Data == MAP_FAILED)
  {
    ErrorMessageFromCall("mmap");
    return -3;
  }
  GPIOAccessControl= (struct GPIOAccessControlStruct *)GPIOSharedMemoryInfo.Data;
  if( GPIOAccessControl->Version != GPIOAccessControlVersion) 
  {
/* This means that shared memory has not been initialized properly, 
 * or it remains from another version of the program.
 * Processing cannot continue.
 */
/*!!!ToDo:
 * Ulepszyc te wiadomosc. Powinna dawac wiecej konkretnych informacji. Jaki segment i jak go wymazac.
 */
    ErrorMessage("Wrong version found in shared memory. It must be initialized again for this program to work. Please, make sure that no programs use the segment and delete it. Afterwards run this probram again.");
    return -10;
  }
  return 0;
}
/* Main initialization function.
 * Must be called first, in order to allow any further actions with this library. */
int GPIOInit( )
{
  int i;
  
  GPIOInitPinMap();
  
  i= GPIOInitHWMemoryAccess();
  if( i)
  {
    ErrorMessageFromCall1( "GPIOInitHWMemoryAccess");
    return -1;
  }

/* Initialize the shared memory structures, used to coordinate concurrent access to GPIO.
 * This library does not destroy the created shared memeory structure when process is exiting. 
 * So proper sequence here is: 1. Try to initiaize. 2. Attach, if the segment already exists.
 */
  i= GPIOSharedMemoryInfoStructureInit(); /* Currently, this function always succeeds... */
  i= GPIOSharedMemoryInit();
  if(i == 1)
  {
    i= GPIOSharedMemoryAttach();
    if( i)
    {
      ErrorMessageFromCall1( "GPIOSharedMemoryAttach");
      return -2;
    }
  } else if( i < 0)
  {
    ErrorMessageFromCall1( "GPIOSharedMemoryInit");
    return -2;
  }
  return 0;
}


/* Map the memory areas. */
/* GPIO1 supported  */
int GPIOInitHWMemoryAccess()
{
  int fd;
  
  if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) 
  {
    ErrorMessageFromCall( "open" );  
    return -1;
  }
    /* Map PMUCRU */
  PMUCRU_base = mmap(0, MAP_64k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF750000);
  if(PMUCRU_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
  /* Map PMUGRF */
  PMUGRF_base = mmap(0, MAP_64k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF320000);
  if(PMUGRF_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
  /* Map GRF */
  GRF_base = mmap(0, MAP_64k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF770000);
  if(GRF_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
  /* Map CRU */
  CRU_base = mmap(0, MAP_64k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF760000);
  if(CRU_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
/* Map PMU */
  PMU_base = mmap(0, MAP_64k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF310000);
  if(PMU_base == (void *) -1)
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
/* Map GPIO1 */
  GPIO1_base = mmap(0, MAP_64k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF730000);
  if(GPIO1_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
/* Map GPIO3 */
  GPIO3_base = mmap(0, MAP_32k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF788000);
  if(GPIO3_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
/* Map GPIO4 */
  GPIO4_base = mmap(0, MAP_32k, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xFF790000);
  if(GPIO4_base == (void *) -1) 
  {
    ErrorMessageFromCall( "mmap");  
    return -1;
  }
  
  return 0;

}


/*  Set multiplexer to the requested mode.
 *   This function enables also clock */

/* The functions setting multiplexer implement only subset of pins, 
 * and subset of pin functionalities. In general, only pins exposed in 
 * the GPIO connector are implemented.
 * 
 * Return values:
 *   0 - Success.
 *  -1 - Pin not implemented.
 *  -2 - function (mode) not implemented.
 *  -10 - Unexpected place in coding has been reached.
 * 
 * Register used:
 *   PMUGRF_GPIO1A_IOMUX  offset 0x00010 W
 */
int GPIOPinModeSetGPIO1_A( int Pin, enum GPIOPinMode Mode)
{
  uint32_t W32= 0;

  if( Pin == 1)
  {
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00010)= W32;
      return 0;
    } else 
    {
/* ISP0 looks like set of lines to controll camera. Only GPIO1_A1 and GPIO1_A4 are exposed from the  
 * ISP0 set. Other lines are not accessible on the RockPro64 board.
 * The same deals with ISP1.
 * Both these modes, ISP0 and ISP1 are not implemented in this library. 
 * So only GPIO mode is imlmented for this pin. */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_A is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }    
  } else if( Pin == 4)
  {
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00010)= W32;
      return 0;
    } else 
    {
/* ISP0 looks like set of line to controll camera. Only GPIO1_A1 and GPIO1_A4 are exposed from the set 
 * ISP0. Other lines are not accessible on the RockPro64 board.
 * The same deals with ISP1.
 * Both these modes, ISP0 and ISP1 are not implemented in this library. 
 * So only GPIO mode is imlmented for this pin. */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_A is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
  } else if( Pin == 7)
  {
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00010)= W32;
      return 0;
    } else if( Mode == GPIOPinModeUART4 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00010)= W32;
      return 0;
    } else if( Mode == GPIOPinModeSPI1 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 2 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00010)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_A is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
  } else       
  {
    fprintf(stderr, "%s() (file %s, line %d): Error: Pin %d of the GPIO1_A is not exposed"
                    " in the GPIO connector, and not implemented in this library.\n", \
             __func__, __FILE__, __LINE__, Pin );
    return -1;
  }
  return -10;
}

/*  Set multiplexer to the requested mode for GPIO1_B*/

/* The functions is like GPIOPinModeSetGPIO1_A() for the GPIO1_A.
 * 
 * Return values:
 *   0 - Success.
 *  -1 - Pin not implemented.
 *  -2 - function (mode) not implemented.
 *  -10 - Unexpected place in coding has been reached.
 * 
 * Register used:
 *   PMUGRF_GPIO1B_IOMUX  offset 0x00014 W
 */
int GPIOPinModeSetGPIO1_B( int Pin, enum GPIOPinMode Mode)
{
  uint32_t W32= 0;

  switch( Pin)
  {
  case 0:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else if( Mode == GPIOPinModeUART4 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else if( Mode == GPIOPinModeSPI1 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 2 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_B is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 1:
  case 2:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else if( Mode == GPIOPinModePMUM0JTAG )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else if( Mode == GPIOPinModeSPI1 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 2 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_B is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 3:
  case 4:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else if( Mode == GPIOPinModeI2C4 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_B is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 5:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00014)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_B is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  default:
    fprintf(stderr, "%s() (file %s, line %d): Error: Pin %d of the GPIO1_B is not exposed"
                    " in the GPIO connector, and not implemented in this library.\n", \
             __func__, __FILE__, __LINE__, Pin );
    return -1;
    break;
  }

  return -10;
}

/*  Set multiplexer to the requested mode for GPIO1_C*/

/* The functions is like GPIOPinModeSetGPIO1_A() for the GPIO1_A.
 * 
 * Return values:
 *   0 - Success.
 *  -1 - Pin not implemented.
 *  -2 - function (mode) not implemented.
 *  -10 - Unexpected place in coding has been reached.
 * 
 * Register used:
 *   PMUGRF_GPIO1C_IOMUX  offset 0x00018 W
 */
int GPIOPinModeSetGPIO1_C( int Pin, enum GPIOPinMode Mode)
{
  uint32_t W32= 0;

  switch( Pin)
  {
  case 2:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00018)= W32;
      return 0;
    } else if( Mode == GPIOPinModeSPI3 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00018)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_C is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 4:
  case 5:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00018)= W32;
      return 0;
    } else if( Mode == GPIOPinModeI2C8 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( PMUGRF_base+ 0x00018)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_C is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 6:
/* Since the other line of tcpdusb2 is not exposed, we do not implement
 * tcpdusb2 for this pin, as well. Only GPIO for this pin.
 */
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( PMUGRF_base+ 0x00018)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO1_C is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  default:
    fprintf(stderr, "%s() (file %s, line %d): Error: Pin %d of the GPIO1_C is not exposed"
                    " in the GPIO connector, and not implemented in this library.\n", \
             __func__, __FILE__, __LINE__, Pin );
    return -1;
    break;
  }

  return -10;
}


/*  Set multiplexer to the requested mode for GPIO3_D*/

/* The functions is like GPIOPinModeSetGPIO1_A() for the GPIO1_A.
 * 
 * Return values:
 *   0 - Success.
 *  -1 - Pin not implemented.
 *  -2 - function (mode) not implemented.
 *  -10 - Unexpected place in coding has been reached.
 * 
 * Register used:
 *   GRF_GPIO3D_IOMUX   offset 0x0e01c W
 */
int GPIOPinModeSetGPIO3_D( int Pin, enum GPIOPinMode Mode)
{
  uint32_t W32= 0;

  switch( Pin)
  {
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( GRF_base+ 0x0e01c)= W32;
      return 0;
    } else if( Mode == GPIOPinModeI2S0 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( GRF_base+ 0x0e01c)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO3_D is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  default:
    fprintf(stderr, "%s() (file %s, line %d): Error: Pin %d of the GPIO3_D is not exposed"
                    " in the GPIO connector, and not implemented in this library.\n", \
             __func__, __FILE__, __LINE__, Pin );
    return -1;
    break;
  }

  return -10;
}

/*  Set multiplexer to the requested mode for GPIO4_C*/

/* The functions is like GPIOPinModeSetGPIO1_A() for the GPIO1_A.
 * 
 * Return values:
 *   0 - Success.
 *  -1 - Pin not implemented.
 *  -2 - function (mode) not implemented.
 *  -10 - Unexpected place in coding has been reached.
 * 
 * Register used:
 *   GRF_GPIO4C_IOMUX   offset 0x0e028 W
 */
int GPIOPinModeSetGPIO4_C( int Pin, enum GPIOPinMode Mode)
{
  uint32_t W32= 0;

  switch( Pin)
  {
  case 3:
  case 4:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else if( Mode == GPIOPinModeUART2 )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else if( Mode == GPIOPinModeUARTHDCP )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 2 << (2* Pin);
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO4_C is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 5:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else if( Mode == GPIOPinModeSPDIF )
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO4_C is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 6:    /* This pin is exposed as J8 (FAN) connector on the ROCKPro64 board. This is the PWM channel 1 (PWM1) */
/*!!!ToDo:
 * Check:
 *  1. If GPIO4_C6 is really exposed at J8 connector.
 *  2. What voltage is has.
 *  3. Is it somehow hardwired or default to PWM.
 */
    if( Mode == GPIOPinModeGPIO )
    {
/* Well, this will be using the J8 connector as GPIO. Probably not intended by board 
 * designer, but why not? 
 * Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else if( Mode == GPIOPinModePWM )  /* This is the intended usage! */
    {
/* Set the write enable bits */  
      W32= 3UL << (16+ (2* Pin));
      W32 |= 1 << (2* Pin);
      *(volatile uint32_t *)( GRF_base+ 0x0e028)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO4_C is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  default:
    fprintf(stderr, "%s() (file %s, line %d): Error: Pin %d of the GPIO4_C is not exposed"
                    " in the GPIO connector, and not implemented in this library.\n", \
             __func__, __FILE__, __LINE__, Pin );
    return -1;
    break;
  }

  return -10;
}

/*  Set multiplexer to the requested mode for GPIO4_D*/

/* The functions is like GPIOPinModeSetGPIO1_A() for the GPIO1_A.
 * 
 * Return values:
 *   0 - Success.
 *  -1 - Pin not implemented.
 *  -2 - function (mode) not implemented.
 *  -10 - Unexpected place in coding has been reached.
 * 
 * Register used:
 *   GRF_GPIO4D_IOMUX   offset 0x0e02c W
 */
int GPIOPinModeSetGPIO4_D( int Pin, enum GPIOPinMode Mode)
{
  uint32_t W32= 0;

  switch( Pin)
  {
  case 0:
  case 1:
    if( Mode == GPIOPinModeGPIO )
    {
/* Set the write enable bits. Only bits are affected, which have the 'write enable' bits set in the upper part
 * of the register. Thus no mutex is necessary for this register. Just write. */  
      W32= 3UL << (16+ (2* Pin));
/* GPIO is selected on the pin (line), when both bits PIN*2 and PIN*2+1 are set to 0.
 * So just write it to the register.
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */  
      *(volatile uint32_t *)( GRF_base+ 0x0e02c)= W32;
      return 0;
    } else
    {
/* No more modes implemented for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO4_D is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    }
    break;
  case 3:
  case 4:  /* According to the TRM document, both these pins cannot have multiplexer set.
              I assume, they are hardwired to GPIO.
            */
/*!!!ToDo:
 * Check if the pins GPIO4_D3 and GPIO4_D4 are allways set to GPIO.
 */
    if( Mode != GPIOPinModeGPIO )
    {
/* No more modes are possible for this pin! */  
      fprintf(stderr, "%s() (file %s, line %d): Error: Mode %d of the GPIO4_D is not implemented for pin %d\n", 
               __func__, __FILE__, __LINE__, Mode, Pin );
      return -2;
    } else return 0;
    break;
  default:
    fprintf(stderr, "%s() (file %s, line %d): Error: Pin %d of the GPIO4_D is not exposed"
                    " in the GPIO connector, and not implemented in this library.\n", \
             __func__, __FILE__, __LINE__, Pin );
    return -1;
    break;
  }

  return -10;
}

/*  Maintain clock for GPIO. Clock is necessary in order to change GPIO line state.
 * After changing, the state is preserved. 
 * Current implementation of sysfs interface is handling clocks differently for different 
 * GPIO blocks. GPIO1 has clock allways enabled. while e.g. GPIO4 gets clock enabled during change
 * or access to a pin, and disabled afterwards. Comments in coding suggest that this saves energy.
 *  Maybe clock of GPIO1 is necessary for some other functions, which are allways enabled? 
 * 
 *   Anyhow, this library enables clock when initialising a pin. Afterwards the clock is not maintained
 * until closing access to the pin. Then the clock is disabled, if it was originally in the disabled state.
 * This saves some time when changing state or reading the pin. Slightly higher power consumption is the expense.
 */

 /*!!!Attention: 
  * 1. Accessing the pin through sysfs interface may disable the clock, and corrupt further
  *    accesses by this library functions. It should be avoided to intermix sysfs accesses and accesses with this library.
  * 2. There is one clock for the complete GPIO block. Initialization of multiple pins from 
  *    the same GPIO block will cause multiple invocation of this function. This is not 
  *    an error, and each invocation will increase the registration counter. 
  *    It is necessary to release each pin, which was initialized, when the pin is not used anymore. 
  *    During this release, the registration counter is decreased.
  *    If you decide not to use standard pin initialisation-release sequence, you need to take care
  *    that the function GPIOClockReturnToPreviousGPIO1() is called finally as many times 
  *    as this function. Else the clock will never return to its original state.
  */

/* For GPIO1:
 * Enable clock for the whole GPIO1. 
 * Writting to this register changes only bits, which are selected in its upper part.
 * So no need to care of other values. Just select, which bits need to be changed, and write their value.
 * Clock is enabled, when the respective bit is 0.
 * It is necessary to remember the revious state of the clock-disable bit. It will be returned
 * to this state by a call to function GPIOClockReturnToPreviousGPIO1()
 * This bit is called pclk_gpio1_en in the TRM document
 * 
 * Register used:
 *   PMUCRU_CLKGATE_CON1  offset 0x0104 W
 */
/*----struct GPIOClockAccessControl
{
  pthread_mutex_t Mutex;
  int ProcessesUsingCurrentState;
  enum GPIOClockState OriginalState;
  enum GPIOClockState CurrentState;  */
 
int GPIOClockEnableGPIO1( )
{
  int i;
  uint32_t W32= 0;

  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. However, assuming that the only
  * actively set state is clock on, there is no risk of using this mutex further. 
  * In the worst case, when a process finished without resetting clock and without decreasing 
  * the process counter, the clock will remain forever on. Not a tragedy...
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  if(  ((GPIOAccessControl->GPIOClockAccessControlGPIO1).ProcessesUsingCurrentState) == 0 ) /* No process has set the clock, so far. */
  {
    W32= *(volatile uint32_t *)( PMUCRU_base+ 0x0104);  
    if( (( W32>>4) &1) == GPIOClockStateEnabled) /* The clock has been on, already.  */
    {
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).OriginalState = GPIOClockStateEnabled;
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).ProcessesUsingCurrentState++;
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).CurrentState = GPIOClockStateEnabled;
    } else
    {
/* Enable clock.
 * Set the write enable bit for the bit 4 */  
      W32= 1UL << (16+ 4);
/* Clock is enabled when the bit is set to 0. But to be consistent with definitions, let us use them:*/  
      W32 |= GPIOClockStateEnabled << 4;
      *(volatile uint32_t *)( PMUCRU_base+ 0x0104)= W32;
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).OriginalState = GPIOClockStateDisabled;
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).ProcessesUsingCurrentState++;
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).CurrentState = GPIOClockStateEnabled;
    }
  } else
  {
    W32= *(volatile uint32_t *)( PMUCRU_base+ 0x0104);  
    if( (( W32>>4) &1) == GPIOClockStateEnabled) /* The clock has been on, already.  */
    {
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).ProcessesUsingCurrentState++;
    } else
    { /* This is really unexpected. Probably error happenned earlier.
         Still to be consistent, we cannot enable clock.
         * Give error message and return error. */
      ErrorMessage( "GPIO1 clock disabled and more than 0 processes using current state. Cannot enable clock. Check if this is not a corrupted state!");
      i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
      if( i )
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -21;
      }
      return -20;
    }
  }
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Clock has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}
/* 
 * Return the clock gate to the original state.
 */
int GPIOClockReturnToPreviousGPIO1( )
{
  int i;
  uint32_t W32= 0;

  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. However, assuming that the only
  * actively set state is clock on, there is no risk of using this mutex further. 
  * In the worst case, when a process finished without resetting clock and without decreasing 
  * the process counter, the clock will remain forever on. Not a tragedy...
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  (GPIOAccessControl->GPIOClockAccessControlGPIO1).ProcessesUsingCurrentState--;
  if( ((GPIOAccessControl->GPIOClockAccessControlGPIO1).ProcessesUsingCurrentState) == 0)
  {
/* Set clock to the original state, because no process is using it anymore.
 * Set the write enable bit for the bit 4 */  
    W32= 1UL << (16+ 4);
/* Original state has ben preserved in the global variable. */  
    W32 |= ((GPIOAccessControl->GPIOClockAccessControlGPIO1).OriginalState) << 4;
    *(volatile uint32_t *)( PMUCRU_base+ 0x0104)= W32;
    (GPIOAccessControl->GPIOClockAccessControlGPIO1).CurrentState = 
      (GPIOAccessControl->GPIOClockAccessControlGPIO1).OriginalState;
  }
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Clock has been handled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Clock for GPIO3.
 * Like for GPIO1. Also protected by mutex in the shared memory.
 * 
 * 
 * Register used:
 *   CRU_CLKGATE_CON31  offset 0x037c W
 * bit 4; pclk_gpio3_en 
 */

int GPIOClockEnableGPIO3( )
{
  int i;
  uint32_t W32= 0;

  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. However, assuming that the only
  * actively set state is clock on, there is no risk of using this mutex further. 
  * In the worst case, when a process finished without resetting clock and without decreasing 
  * the process counter, the clock will remain forever on. Not a tragedy...
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  if(  ((GPIOAccessControl->GPIOClockAccessControlGPIO3).ProcessesUsingCurrentState) == 0 ) /* No process has set the clock, so far. */
  {
    W32= *(volatile uint32_t *)( CRU_base+ 0x037c);  
    if( (( W32>>4) &1) == GPIOClockStateEnabled) /* The clock has been on, already.  */
    {
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).OriginalState = GPIOClockStateEnabled;
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).ProcessesUsingCurrentState++;
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).CurrentState = GPIOClockStateEnabled;
    } else
    {
/* Enable clock.
 * Set the write enable bit for the bit 4 */  
      W32= 1UL << (16+ 4);
/* Clock is enabled when the bit is set to 0. But to be consistent with definitions, let us use them:*/  
      W32 |= GPIOClockStateEnabled << 4;
      *(volatile uint32_t *)( CRU_base+ 0x037c)= W32;
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).OriginalState = GPIOClockStateDisabled;
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).ProcessesUsingCurrentState++;
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).CurrentState = GPIOClockStateEnabled;
    }
  } else
  {
    W32= *(volatile uint32_t *)( CRU_base+ 0x037c);  
    if( (( W32>>4) &1) == GPIOClockStateEnabled) /* The clock has been on, already.  */
    {
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).ProcessesUsingCurrentState++;
    } else
    { /* This is really unexpected. Probably error happenned earlier.
         Still to be consistent, we cannot enable clock.
         * Give error message and return error. */
      ErrorMessage( "GPIO3 clock disabled and more than 0 processes using current state. Cannot enable clock. Check if this is not a corrupted state!");
      i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
      if( i )
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -21;
      }
      return -20;
    }
  }
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Clock has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}
/* 
 * GPIO3: Return the clock gate to the original state.
 */
int GPIOClockReturnToPreviousGPIO3( )
{
  int i;
  uint32_t W32= 0;

  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. However, assuming that the only
  * actively set state is clock on, there is no risk of using this mutex further. 
  * In the worst case, when a process finished without resetting clock and without decreasing 
  * the process counter, the clock will remain forever on. Not a tragedy...
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  (GPIOAccessControl->GPIOClockAccessControlGPIO3).ProcessesUsingCurrentState--;
  if( ((GPIOAccessControl->GPIOClockAccessControlGPIO3).ProcessesUsingCurrentState) == 0)
  {
/* Set clock to the original state, because no process is using it anymore.
 * Set the write enable bit for the bit 4 */  
    W32= 1UL << (16+ 4);
/* Original state has ben preserved in the global variable. */  
    W32 |= ((GPIOAccessControl->GPIOClockAccessControlGPIO3).OriginalState) << 4;
    *(volatile uint32_t *)( CRU_base+ 0x037c)= W32;
    (GPIOAccessControl->GPIOClockAccessControlGPIO3).CurrentState = 
      (GPIOAccessControl->GPIOClockAccessControlGPIO3).OriginalState;
  }
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO3).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Clock has been handled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Clock for GPIO4.
 * Like for GPIO1. Also protected by mutex in the shared memory.
 * 
 * 
 * Register used:
 *   CRU_CLKGATE_CON31  offset 0x037c W
 * bit 5; pclk_gpio4_en
 * 
 */

int GPIOClockEnableGPIO4( )
{
  int i;
  uint32_t W32= 0;

  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. However, assuming that the only
  * actively set state is clock on, there is no risk of using this mutex further. 
  * In the worst case, when a process finished without resetting clock and without decreasing 
  * the process counter, the clock will remain forever on. Not a tragedy...
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  if(  ((GPIOAccessControl->GPIOClockAccessControlGPIO4).ProcessesUsingCurrentState) == 0 ) /* No process has set the clock, so far. */
  {
    W32= *(volatile uint32_t *)( CRU_base+ 0x037c);  
    if( (( W32>>5) &1) == GPIOClockStateEnabled) /* The clock has been on, already.  */
    {
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).OriginalState = GPIOClockStateEnabled;
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).ProcessesUsingCurrentState++;
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).CurrentState = GPIOClockStateEnabled;
    } else
    {
/* Enable clock.
 * Set the write enable bit for the bit 4 */  
      W32= 1UL << (16+ 5);
/* Clock is enabled when the bit is set to 0. But to be consistent with definitions, let us use them:*/  
      W32 |= GPIOClockStateEnabled << 5;
      *(volatile uint32_t *)( CRU_base+ 0x037c)= W32;
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).OriginalState = GPIOClockStateDisabled;
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).ProcessesUsingCurrentState++;
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).CurrentState = GPIOClockStateEnabled;
    }
  } else
  {
    W32= *(volatile uint32_t *)( CRU_base+ 0x037c);  
    if( (( W32>>5) &1) == GPIOClockStateEnabled) /* The clock has been on, already.  */
    {
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).ProcessesUsingCurrentState++;
    } else
    { /* This is really unexpected. Probably error happenned earlier.
         Still to be consistent, we cannot enable clock.
         * Give error message and return error. */
      ErrorMessage( "GPIO4 clock disabled and more than 0 processes using current state. Cannot enable clock. Check if this is not a corrupted state!");
      i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
      if( i )
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -21;
      }
      return -20;
    }
  }
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Clock has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}
/* 
 * GPIO4: Return the clock gate to the original state.
 */
int GPIOClockReturnToPreviousGPIO4( )
{
  int i;
  uint32_t W32= 0;

  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. However, assuming that the only
  * actively set state is clock on, there is no risk of using this mutex further. 
  * In the worst case, when a process finished without resetting clock and without decreasing 
  * the process counter, the clock will remain forever on. Not a tragedy...
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  (GPIOAccessControl->GPIOClockAccessControlGPIO4).ProcessesUsingCurrentState--;
  if( ((GPIOAccessControl->GPIOClockAccessControlGPIO4).ProcessesUsingCurrentState) == 0)
  {
/* Set clock to the original state, because no process is using it anymore.
 * Set the write enable bit for the bit 4 */  
    W32= 1UL << (16+ 5);
/* Original state has ben preserved in the global variable. */  
    W32 |= ((GPIOAccessControl->GPIOClockAccessControlGPIO4).OriginalState) << 5;
    *(volatile uint32_t *)( CRU_base+ 0x037c)= W32;
    (GPIOAccessControl->GPIOClockAccessControlGPIO4).CurrentState = 
      (GPIOAccessControl->GPIOClockAccessControlGPIO4).OriginalState;
  }
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOClockAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Clock has been handled, but error from mutex handling is something severe!  */
  }
  return 0;
}


/*!!!ToDo: 
 * Consider, what are following registers doing, and if they need to be handled:
 *   PMUCRU_SOFTRST_CON1
 *   PMUCRU_RSTNHOLD_CON0
 *  -> PMUGRF_GPIO1L_SR
 *  -> PMUGRF_GPIO1A_SMT
 *   PMU_WAKEUP_CFG2
 *   PMU_WAKEUP_CFG3
 *   PMU_GPIO1_POS_INT_CON
 *   PMU_GPIO1_POS_INT_ST
 *   
 *   GPIO_INTEN
 *   GPIO_INTMASK
 *   GPIO_INTTYPE_LEVEL
 *   GPIO_INT_POLARITY
 *   GPIO_INT_STATUS
 *   GPIO_INT_RAWSTATUS
 *   GPIO_DEBOUNCE
 *   GPIO_LS_SYNC
 * 
 */

/*
 * Drive strength control
 */
 
/* GPIO1_A 
 *  
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register PMUGRF_GPIO1A_E  offset: 0x000a0 W
 */ 
int GPIODriveStrengthGPIO1_A( int Pin, enum GPIODriveStrengthState Current )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIODriveStrengthState contains the proper bits already. There is no 
 * verification in this function. 
 * When writting to this register, only the bits are affected, which are selected 
 * in the upper part of the register, by the write enable bits. 
 * So it is not necessary to take care of other bits.
 * Simply write. */ 
/*!!!ToDo:
 * Dodac zabezpieczenie do tej funkcji. 
 * Moc nie powinna byc zmniejszana. To minimum. Czyli sprawdzic, jaka jest, zanim bedzie ustawiana.
 * Moze dodac jakas mozliwosc powrotu do stanu poprzedniego? Tak jak dla GPIO Clock?
 * A moze zostawic to dla programisty aplikacji?
 */
  W32 |= Current << (2* Pin);
  *(volatile uint32_t *)( PMUGRF_base+ 0x000a0)= W32;
  return 0;
}
/* GPIO1_B
 *  
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register PMUGRF_GPIO1B_E  offset: 0x000a8 W
 */ 
int GPIODriveStrengthGPIO1_B( int Pin, enum GPIODriveStrengthState Current )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIODriveStrengthState contains the proper bits already. There is no 
 * verification in this function. 
 * When writting to this register, only the bits are affected, which are selected 
 * in the upper part of the register, by the write enable bits. 
 * So it is not necessary to take care of other bits.
 * Simply write. */ 
/*!!!ToDo:
 * Dodac zabezpieczenie do tej funkcji. 
 * Moc nie powinna byc zmniejszana. To minimum. Czyli sprawdzic, jaka jest, zanim bedzie ustawiana.
 * Moze dodac jakas mozliwosc powrotu do stanu poprzedniego? Tak jak dla GPIO Clock?
 * A moze zostawic to dla programisty aplikacji?
 */
  W32 |= Current << (2* Pin);
  *(volatile uint32_t *)( PMUGRF_base+ 0x000a8)= W32;
  return 0;
}
/* GPIO1_C
 *  
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register PMUGRF_GPIO1C_E  offset: 0x000b0 W
 */ 
int GPIODriveStrengthGPIO1_C( int Pin, enum GPIODriveStrengthState Current )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIODriveStrengthState contains the proper bits already. There is no 
 * verification in this function. 
 * When writting to this register, only the bits are affected, which are selected 
 * in the upper part of the register, by the write enable bits. 
 * So it is not necessary to take care of other bits.
 * Simply write. */ 
/*!!!ToDo:
 * Dodac zabezpieczenie do tej funkcji. 
 * Moc nie powinna byc zmniejszana. To minimum. Czyli sprawdzic, jaka jest, zanim bedzie ustawiana.
 * Moze dodac jakas mozliwosc powrotu do stanu poprzedniego? Tak jak dla GPIO Clock?
 * A moze zostawic to dla programisty aplikacji?
 */
  W32 |= Current << (2* Pin);
  *(volatile uint32_t *)( PMUGRF_base+ 0x000b0)= W32;
  return 0;
}
/* GPIO3_D
 *  
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register GRF_GPIO3D_E  offset: 0x0e128 W
 */ 
int GPIODriveStrengthGPIO3_D( int Pin, enum GPIODriveStrengthState Current )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIODriveStrengthState contains the proper bits already. There is no 
 * verification in this function. 
 * When writting to this register, only the bits are affected, which are selected 
 * in the upper part of the register, by the write enable bits. 
 * So it is not necessary to take care of other bits.
 * Simply write. */ 
/*!!!ToDo:
 * Dodac zabezpieczenie do tej funkcji. 
 * Moc nie powinna byc zmniejszana. To minimum. Czyli sprawdzic, jaka jest, zanim bedzie ustawiana.
 * Moze dodac jakas mozliwosc powrotu do stanu poprzedniego? Tak jak dla GPIO Clock?
 * A moze zostawic to dla programisty aplikacji?
 */
  W32 |= Current << (2* Pin);
  *(volatile uint32_t *)( GRF_base+ 0x0e128)= W32;
  return 0;
}
/* GPIO4_C
 *  
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register GRF_GPIO4C_E  offset: 0x0e138 W
 */
int GPIODriveStrengthGPIO4_C( int Pin, enum GPIODriveStrengthState Current )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIODriveStrengthState contains the proper bits already. There is no 
 * verification in this function. 
 * When writting to this register, only the bits are affected, which are selected 
 * in the upper part of the register, by the write enable bits. 
 * So it is not necessary to take care of other bits.
 * Simply write. */ 
/*!!!ToDo:
 * Dodac zabezpieczenie do tej funkcji. 
 * Moc nie powinna byc zmniejszana. To minimum. Czyli sprawdzic, jaka jest, zanim bedzie ustawiana.
 * Moze dodac jakas mozliwosc powrotu do stanu poprzedniego? Tak jak dla GPIO Clock?
 * A moze zostawic to dla programisty aplikacji?
 */
  W32 |= Current << (2* Pin);
  *(volatile uint32_t *)( GRF_base+ 0x0e138)= W32;
  return 0;
}
/* GPIO4_D
 *  
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register GRF_GPIO4D_E  offset: 0x0e13c W
 */
int GPIODriveStrengthGPIO4_D( int Pin, enum GPIODriveStrengthState Current )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIODriveStrengthState contains the proper bits already. There is no 
 * verification in this function. 
 * When writting to this register, only the bits are affected, which are selected 
 * in the upper part of the register, by the write enable bits. 
 * So it is not necessary to take care of other bits.
 * Simply write. */ 
/*!!!ToDo:
 * Dodac zabezpieczenie do tej funkcji. 
 * Moc nie powinna byc zmniejszana. To minimum. Czyli sprawdzic, jaka jest, zanim bedzie ustawiana.
 * Moze dodac jakas mozliwosc powrotu do stanu poprzedniego? Tak jak dla GPIO Clock?
 * A moze zostawic to dla programisty aplikacji?
 */
  W32 |= Current << (2* Pin);
  *(volatile uint32_t *)( GRF_base+ 0x0e13c)= W32;
  return 0;
}



/*
 * Manipulation of GPIO lines.
 * 
 * These functions are extremely unsafe in case of concurrent processes accessing 
 * the same GPIO block!
 * 
 * All lines of a block are handled by one register. There is no mechanism of writting enable
 * for particular bits. Thus, register value must be first read, then the relevant bit is changed 
 * and subsequently the whole register is writtne back. The functions below perform 
 * no locking, which could avoid concurrent changes.
 */

/*!!!ToDo:
 * Write the unlocked (not protected by mutex) fuctions, or macros.
 */

/* The functions below implement mutexes in shared memory. In order to protect 
 * against race conditions when accessing register concurrently.
 */

/* Write 0 or 1 to a pin of GPIO1_A. This function does not control the direction
 * of the pin. Just writes.
 */
int GPIOSetPinStateGPIO1_A( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  
  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << Pin;
  } else
  {
    W32 &= ~(1UL << Pin);
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0000)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}
/* Write 0 or 1 to a pin of GPIO1_B. This function does not control the direction
 * of the pin. Just writes.
 */
int GPIOSetPinStateGPIO1_B( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  
  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 8);
  } else
  {
    W32 &= ~(1UL << (Pin+ 8));
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0000)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}
/* Write 0 or 1 to a pin of GPIO1_C. This function does not control the direction
 * of the pin. Just writes.
 */
int GPIOSetPinStateGPIO1_C( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  
  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 16);
  } else
  {
    W32 &= ~(1UL << (Pin+ 16));
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0000)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}
/* Write 0 or 1 to a pin of GPIO3_D. This function does not control the direction
 * of the pin. Just writes.
 */
int GPIOSetPinStateGPIO3_D( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO3).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOSetStateAccessControlGPIO3).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  
  W32= *(volatile uint32_t *)( GPIO3_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 24);
  } else
  {
    W32 &= ~(1UL << (Pin+ 24));
  }
  *(volatile uint32_t *)( GPIO3_base+ 0x0000)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO3).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}

/* Write 0 or 1 to a pin of GPIO4_C. This function does not control the direction
 * of the pin. Just writes.
 */
int GPIOSetPinStateGPIO4_C( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  
  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 16);
  } else
  {
    W32 &= ~(1UL << (Pin+ 16));
  }
  *(volatile uint32_t *)( GPIO4_base+ 0x0000)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}
/* Write 0 or 1 to a pin of GPIO4_D. This function does not control the direction
 * of the pin. Just writes.
 */
int GPIOSetPinStateGPIO4_D( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }
  
  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 24);
  } else
  {
    W32 &= ~(1UL << (Pin+ 24));
  }
  *(volatile uint32_t *)( GPIO4_base+ 0x0000)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}

/* The 'unlocked' versions of the GPIOSetPinStateXXX functions.
 * These functions are provided to achieve higher speed in extreme cases,
 * because they do not perform mutex operations.
 * These fuctions are not protected against concurrent access to the GPIO
 * blocks. So they should not be used, if any possibility of concurrent access
 * exists. 
 * Alternatively, permanent locking of pin functions may be used with functions GPIOLock / GPIOUnlock
 * 
 * The functions still read content of the register, in order to preserver state of other pins.
 * This reading may be performed once before first invocation, and stored in a global
 * veriable. This is may yiedl even more speed.
 * 
 */ 
 
/*!!!ToDo:
 * Measuere performance gain, when using the below functions instead of the ones with mutex protection.
 * Measure performance gain, when not reading the register content before writting (like proposed just above).
 */

int GPIOSetPinStateGPIO1_A_unlocked( int Pin, enum GPIOPinState State )
{
  uint32_t W32;

  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << Pin;
  } else
  {
    W32 &= ~(1UL << Pin);
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0000)= W32;
  
  return 0;
}

int GPIOSetPinStateGPIO1_B_unlocked( int Pin, enum GPIOPinState State )
{
  uint32_t W32;

  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 8);
  } else
  {
    W32 &= ~(1UL << (Pin+ 8));
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0000)= W32;

  return 0;
}

int GPIOSetPinStateGPIO1_C_unlocked( int Pin, enum GPIOPinState State )
{
  uint32_t W32;

  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 16);
  } else
  {
    W32 &= ~(1UL << (Pin+ 16));
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0000)= W32;
  
  return 0;
}

int GPIOSetPinStateGPIO3_D_unlocked( int Pin, enum GPIOPinState State )
{
  uint32_t W32;

  W32= *(volatile uint32_t *)( GPIO3_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 24);
  } else
  {
    W32 &= ~(1UL << (Pin+ 24));
  }
  *(volatile uint32_t *)( GPIO3_base+ 0x0000)= W32;

  return 0;
}

int GPIOSetPinStateGPIO4_C_unlocked( int Pin, enum GPIOPinState State )
{
  uint32_t W32;

  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 16);
  } else
  {
    W32 &= ~(1UL << (Pin+ 16));
  }
  *(volatile uint32_t *)( GPIO4_base+ 0x0000)= W32;
    
  return 0;
}

int GPIOSetPinStateGPIO4_D_unlocked( int Pin, enum GPIOPinState State )
{
  uint32_t W32;
  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0000);
  if( State)
  {
    W32 |= 1UL << (Pin+ 24);
  } else
  {
    W32 &= ~(1UL << (Pin+ 24));
  }
  *(volatile uint32_t *)( GPIO4_base+ 0x0000)= W32;
  return 0;
}


/* Read state of  a pin of GPIO1_A. This function does not control the direction
 * of the pin. So it may return current external signal applied to input pin,
 * or a state previously set for output pin.
 * 
 * No protection against concurrent acces is applied. Only one pin is returned 
 * We assume that one hardware pin is used for one purpose by one program.
 * 
 */
enum GPIOPinState GPIOGetPinStateGPIO1_A( int Pin )
{
  uint32_t W32;
  
  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0050) ;
  return (( W32>>(Pin)) &1);
}
/* Read state of  a pin of GPIO1_B. This function does not control the direction
 * of the pin. So it may return current external signal applied to input pin,
 * or a state previously set for output pin.
 * 
 * No protection against concurrent acces is applied. Only one pin is returned 
 * We assume that one hardware pin is used for one purpose by one program.
 * 
 */
enum GPIOPinState GPIOGetPinStateGPIO1_B( int Pin )
{
  uint32_t W32;
  
  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0050) ;
  return (( W32>>(Pin+ 8)) &1);
}
/* Read state of  a pin of GPIO1_C. This function does not control the direction
 * of the pin. So it may return current external signal applied to input pin,
 * or a state previously set for output pin.
 * 
 * No protection against concurrent acces is applied. Only one pin is returned 
 * We assume that one hardware pin is used for one purpose by one program.
 * 
 */
enum GPIOPinState GPIOGetPinStateGPIO1_C( int Pin )
{
  uint32_t W32;
  
  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0050) ;
  return (( W32>>(Pin+ 16)) &1);
}
/* Read state of  a pin of GPIO3_D. This function does not control the direction
 * of the pin. So it may return current external signal applied to input pin,
 * or a state previously set for output pin.
 * 
 * No protection against concurrent acces is applied. Only one pin is returned 
 * We assume that one hardware pin is used for one purpose by one program.
 * 
 */
enum GPIOPinState GPIOGetPinStateGPIO3_D( int Pin )
{
  uint32_t W32;
  
  W32= *(volatile uint32_t *)( GPIO3_base+ 0x0050) ;
  return (( W32>>(Pin+ 24)) &1);
}
/* Read state of  a pin of GPIO4_C. This function does not control the direction
 * of the pin. So it may return current external signal applied to input pin,
 * or a state previously set for output pin.
 * 
 * No protection against concurrent acces is applied. Only one pin is returned 
 * We assume that one hardware pin is used for one purpose by one program.
 * 
 */
enum GPIOPinState GPIOGetPinStateGPIO4_C( int Pin )
{
  uint32_t W32;
  
  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0050) ;
  return (( W32>>(Pin+ 16)) &1);
}
/* Read state of  a pin of GPIO4_D. This function does not control the direction
 * of the pin. So it may return current external signal applied to input pin,
 * or a state previously set for output pin.
 * 
 * No protection against concurrent acces is applied. Only one pin is returned 
 * We assume that one hardware pin is used for one purpose by one program.
 * 
 */
enum GPIOPinState GPIOGetPinStateGPIO4_D( int Pin )
{
  uint32_t W32;
  
  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0050) ;
  return (( W32>>(Pin+ 24)) &1);
}

/* Set direction of a pin of GPIO1_A. 
 */
int GPIOSetPinDirectionGPIO1_A( int Pin, enum GPIOPinDirection Dir)
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }

  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0004) ;
  if( Dir)
  {
    W32 |= 1UL << Pin;
  } else
  {
    W32 &= ~(1UL << Pin);
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0004)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Set direction of a pin of GPIO1_B. 
 * The pins of GPIO1_B are in the same GPIO registers, as GPIO1_A. Just shifted by 8. 
 */
int GPIOSetPinDirectionGPIO1_B( int Pin, enum GPIOPinDirection Dir)
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }

  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0004) ;
  if( Dir)
  {
    W32 |= 1UL << ( Pin + 8);
  } else
  {
    W32 &= ~(1UL << ( Pin+ 8));
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0004)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Set direction of a pin of GPIO1_C. 
 * The pins of GPIO1_C are in the same GPIO registers, as GPIO1_A. Just shifted by 16. 
 */
int GPIOSetPinDirectionGPIO1_C( int Pin, enum GPIOPinDirection Dir)
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }

  W32= *(volatile uint32_t *)( GPIO1_base+ 0x0004) ;
  if( Dir)
  {
    W32 |= 1UL << ( Pin + 16);
  } else
  {
    W32 &= ~(1UL << ( Pin+ 16));
  }
  *(volatile uint32_t *)( GPIO1_base+ 0x0004)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Set direction of a pin of GPIO3_D. 
 * The pins of GPIO3_D are in the same GPIO registers, as GPIO3_A. Just shifted by 24. 
 */
int GPIOSetPinDirectionGPIO3_D( int Pin, enum GPIOPinDirection Dir)
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO3).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIODirectionAccessControlGPIO3).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }

  W32= *(volatile uint32_t *)( GPIO3_base+ 0x0004) ;
  if( Dir)
  {
    W32 |= 1UL << ( Pin + 24);
  } else
  {
    W32 &= ~(1UL << ( Pin+ 24));
  }
  *(volatile uint32_t *)( GPIO3_base+ 0x0004)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO3).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Set direction of a pin of GPIO4_C. 
 * The pins of GPIO4_C are in the same GPIO registers, as GPIO4_A. Just shifted by 16. 
 */
int GPIOSetPinDirectionGPIO4_C( int Pin, enum GPIOPinDirection Dir)
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO4).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIODirectionAccessControlGPIO4).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }

  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0004) ;
  if( Dir)
  {
    W32 |= 1UL << ( Pin + 16);
  } else
  {
    W32 &= ~(1UL << ( Pin+ 16));
  }
  *(volatile uint32_t *)( GPIO4_base+ 0x0004)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}
/* Set direction of a pin of GPIO4_D. 
 * The pins of GPIO4_D are in the same GPIO registers, as GPIO4_A. Just shifted by 24. 
 */
int GPIOSetPinDirectionGPIO4_D( int Pin, enum GPIOPinDirection Dir)
{
  uint32_t W32;
  int i;
  
  i= pthread_mutex_lock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO4).Mutex));
  if( (i != 0) )
  {
    if(i == EOWNERDEAD) 
    {
 /* Currently, there is no way to make this state really consistent. 
  * However, there should not be any risk of using this mutex further. 
  * The only function of this mutex is to impose exclusive access during setting
  * this register. 
  * The only problem may arise, when more processes try to recoer this mutex concurrently.
  * Let us deal later, with this potential issue.
  * 
  */
      fprintf( stderr, "%s() (file %s, line %d): Warning: pthread_mutex_lock() returned EOWNERDEAD. "
                       "The previous mutex owner ended without ulocking the mutex.\n",
                __func__, __FILE__, __LINE__);
/*!!!ToDo:
 * Find way to protect against concurrent execution of this function.
 * Or test that it makes no harm.
 */
      i= pthread_mutex_consistent( &((GPIOAccessControl->GPIODirectionAccessControlGPIO4).Mutex));
      if( i)   
      {
        fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_consistent(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
        return -1;
      }
    } 
    else
    {
      fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_lock(). The function returned: %d\n",
                  __func__, __FILE__, __LINE__, i);
      return -10;
    }
  }

  W32= *(volatile uint32_t *)( GPIO4_base+ 0x0004) ;
  if( Dir)
  {
    W32 |= 1UL << ( Pin + 24);
  } else
  {
    W32 &= ~(1UL << ( Pin+ 24));
  }
  *(volatile uint32_t *)( GPIO4_base+ 0x0004)= W32;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIODirectionAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* Functions, which operate with pin numbers referring to the 40-pin GPIO connector 
 * on ROCKPro64 board.
 */ 

/* This function should be called before any operation with a pin.
 * It sets the multiplexer and clock properly.  
 */
int GPIOPinInit( int Pin, enum GPIOPinMode Mode)
{
  int RC;
  
  if( Pin > 40 || Pin < 0 || (GPIOPinMap[Pin].Pin == -1) )
  {
    ErrorMessage( "Pin not implemented in this library, or out of range.");
    return -1;
  }
  
  if( GPIOPinMap[Pin].ModeSet == Mode) return 0;
  if( GPIOPinMap[Pin].ModeSet != GPIOPinModeInvalid)
  {
    fprintf(stderr, "%s() (file %s, line %d): Error: "
                    "Mode %d requested for pin %d, but the pin has been already "
                    "set by this process to mode: %d\n",
            __func__, __FILE__, __LINE__, 
            Mode, Pin, (GPIOPinMap[Pin].ModeSet) );
    return -4;
  }
  if( Mode == GPIOPinModeGPIO)
  {
    RC= (GPIOPinMap[Pin].GPIOClockEnable)();
    if( RC < 0)
    {
      ErrorMessageFromCall1( "GPIOClockEnable");
      return -2;
    }
  }
  RC= (GPIOPinMap[Pin].GPIOPinModeSet)( GPIOPinMap[Pin].Pin, Mode);
  if( RC < 0)
  {
    ErrorMessageFromCall1( "GPIOPinModeSet");
    return -3;
  }
  GPIOPinMap[Pin].ModeSet= Mode;
  return 0;
} 

/* This function should be called after operations with a pin are finished.
 * It reverts setting, which have been done by GPIOPinInit().
 * After this call, the pin may not be usable, until GPIOPinInit() is called for the pin again.
 */
int GPIOPinRelease( int Pin)
{
  int RC;
  
  if( Pin > 40 || Pin < 0 || (GPIOPinMap[Pin].Pin == -1) )
  {
    ErrorMessage( "Pin not implemented in this library, or out of range.");
    return -1;
  }
/* Actually, GPIO is the only mode for which we are doing anything here. So far.  */  
  if( GPIOPinMap[Pin].ModeSet == GPIOPinModeGPIO) 
  {
    RC= (GPIOPinMap[Pin].GPIOClockReturnToPrevious)();
    if( RC < 0)
    {
      ErrorMessageFromCall1( "GPIOClockReturnToPrevious");
      return -2;
    }
  } 
  GPIOPinMap[Pin].ModeSet= GPIOPinModeInvalid; 
  return 0;
}

int GPIOSetPinDirection( int Pin, enum GPIOPinDirection Dir)
{
  int RC;
  
  if( Pin > 40 || Pin < 0 || (GPIOPinMap[Pin].Pin == -1) )
  {
    ErrorMessage( "Pin not implemented in this library, or out of range.");
    return -1;
  }
  
  RC= (GPIOPinMap[Pin].GPIOSetPinDirection)( GPIOPinMap[Pin].Pin, Dir);
  if( RC < 0)
  {
    ErrorMessageFromCall1( "GPIOSetPinDirection");
    return -2;
  }
  return 0;
}

int GPIOSetPullUpDown( int Pin, enum GPIOPullUpDownState Pull )
{
  int RC;
  
  if( Pin > 40 || Pin < 0 || (GPIOPinMap[Pin].Pin == -1) )
  {
    ErrorMessage( "Pin not implemented in this library, or out of range.");
    return -1;
  }
  
  RC= (GPIOPinMap[Pin].GPIOSetPullUpDown)( GPIOPinMap[Pin].Pin, Pull);
  if( RC < 0)
  {
    ErrorMessageFromCall1( "GPIOSetPullUpDown");
    return -2;
  }
  return 0;
}

enum GPIOPullUpDownState GPIOGetPullUpDown( int Pin )
{

  if( Pin > 40 || Pin < 0 || (GPIOPinMap[Pin].Pin == -1) )
  {
    ErrorMessage( "Pin not implemented in this library, or out of range.");
    return -1;
  }

  return (GPIOPinMap[Pin].GPIOGetPullUpDown)( GPIOPinMap[Pin].Pin);
}

int GPIODriveStrength( int Pin, enum GPIODriveStrengthState Current )
{
  int RC;
  
  if( Pin > 40 || Pin < 0 || (GPIOPinMap[Pin].Pin == -1) )
  {
    ErrorMessage( "Pin not implemented in this library, or out of range.");
    return -1;
  }
  
  RC= (GPIOPinMap[Pin].GPIODriveStrength)( GPIOPinMap[Pin].Pin, Current);
  if( RC < 0)
  {
    ErrorMessageFromCall1( "GPIODriveStrength");
    return -2;
  }
  return 0;
}


/* The functions below try to achieve highest speed of execution. 
 * So they do not perform several checks. E.g. no check for correct pin number. 
 */
int GPIOSetPinState( int Pin, enum GPIOPinState State )
{
  int RC;

  RC= (GPIOPinMap[Pin].GPIOSetPinState)( GPIOPinMap[Pin].Pin, State);
  if( RC < 0)
  {
    ErrorMessageFromCall1( "GPIOSetPinState");
    return -2;
  }
  return 0;
}
int GPIOSetPinState_unlocked( int Pin, enum GPIOPinState State )
{
/* The _unlocked functions allways return 0. So does this function  */
  (GPIOPinMap[Pin].GPIOSetPinState_unlocked)( GPIOPinMap[Pin].Pin, State);
  return 0;
}

enum GPIOPinState GPIOGetPinState( int Pin )
{
  return (GPIOPinMap[Pin].GPIOGetPinState)( GPIOPinMap[Pin].Pin);
}

/* The functions to handle permanent lock on access to pin functions.
 * Be very careful: They lock some functions of all pins in the same GPIO block!
 */  
 
/* Remember to unlock eventualy, if you have locked!  */
int GPIOLock( int Pin )
{
  return (GPIOPinMap[Pin].GPIOLock)();
}

int GPIOUnlock( int Pin )
{
  return (GPIOPinMap[Pin].GPIOLock)();
}

