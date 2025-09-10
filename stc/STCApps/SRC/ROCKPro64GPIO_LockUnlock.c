/* 
 * Functions to handle GPIO connector on the ROCKPro64 board.
 * 
 * In this file: 
 * Locking and ulocking GPIO block
 * This locking is perfomed transparently in functions performing actions, which require such lock.
 * It means, functions which update registers, containing multiple control bits, what may
 * create race conditions, or functions which change functionality affecting multiple 
 * pins, like GPIO clock.
 * 
 * Locking by separate function may be used in special cases for extreme performance or stability. 
 * It needs to be used carefully, because such lock makes several pins not accessible to other
 * applications. Also the lock must be released explicitely.  
 */


#include "ROCKPro64GPIO.h"
#include "ROCKPro64GPIO_Globals.h"

/* For pins of block GPIO1  */
int GPIOLockGPIO1( )
{
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
    
  return 0;
}

int GPIOUnlockGPIO1( )
{
  int i;
  
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO1).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  return 0;
}

/* For pins of block GPIO3  */

int GPIOLockGPIO3( )
{
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
  
  return 0;
}
  
int GPIOUnlockGPIO3( )
{
  int i;
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO3).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}


/* For pins of block GPIO4  */
int GPIOLockGPIO4( )
{
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
  
  return 0;
}

int GPIOUnlockGPIO4( )
{
  int i;
  i= pthread_mutex_unlock(&((GPIOAccessControl->GPIOSetStateAccessControlGPIO4).Mutex));
  if( i )
  {
    fprintf( stderr, "%s() (file %s, line %d): Error from pthread_mutex_unlock(). The function returned: %d\n",
              __func__, __FILE__, __LINE__, i);
    return -30;   /* Proper direction has been enabled, but error from mutex handling is something severe!  */
  }
  
  return 0;
}

