/* 
 * Functions to handle GPIO connector on the ROCKPro64 board.
 * 
 * In this file: 
 * Pull-up and pull-down functionality
 */


#include "ROCKPro64GPIO.h"
#include "ROCKPro64GPIO_Globals.h"

/* Set pull-up or pull-down of a GPIO pin*/


/* GPIO1_A
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register PMUGRF_GPIO1A_P offset: 0x00050 W
 */
int GPIOSetPullUpDownGPIO1_A( int Pin, enum GPIOPullUpDownState Pull )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIOPullUpDownState contains the proper bits already. There is no 
 * verification in this function. 
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */ 
  W32 |= Pull << (2* Pin);
  *(volatile uint32_t *)( PMUGRF_base+ 0x00050)= W32;
  return 0;
}
/* GPIO1_B
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register PMUGRF_GPIO1B_P   offset: 0x00054 W
 */
int GPIOSetPullUpDownGPIO1_B( int Pin, enum GPIOPullUpDownState Pull )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIOPullUpDownState contains the proper bits already. There is no 
 * verification in this function. 
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */ 
  W32 |= Pull << (2* Pin);
  *(volatile uint32_t *)( PMUGRF_base+ 0x00054)= W32;
  return 0;
}
/* GPIO1_C
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register PMUGRF_GPIO1C_P  offset: 0x00058 W
 */
int GPIOSetPullUpDownGPIO1_C( int Pin, enum GPIOPullUpDownState Pull )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIOPullUpDownState contains the proper bits already. There is no 
 * verification in this function. 
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */ 
  W32 |= Pull << (2* Pin);
  *(volatile uint32_t *)( PMUGRF_base+ 0x00058)= W32;
  return 0;
}
/* GPIO3_D
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register GRF_GPIO3D_P  offset: 0x0e05c W
 */
int GPIOSetPullUpDownGPIO3_D( int Pin, enum GPIOPullUpDownState Pull )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIOPullUpDownState contains the proper bits already. There is no 
 * verification in this function. 
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */ 
  W32 |= Pull << (2* Pin);
  *(volatile uint32_t *)( GRF_base+ 0x0e05c)= W32;
  return 0;
}
/* GPIO4_C
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register GRF_GPIO4C_P  offset: 0x0e068 W
 */
int GPIOSetPullUpDownGPIO4_C( int Pin, enum GPIOPullUpDownState Pull )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIOPullUpDownState contains the proper bits already. There is no 
 * verification in this function. 
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */ 
  W32 |= Pull << (2* Pin);
  *(volatile uint32_t *)( GRF_base+ 0x0e068)= W32;
  return 0;
}
/* GPIO4_D
 * This function changes only bits of one pin. So no protection against concurrent
 * access is necessary.  
 * 
 * Register GRF_GPIO4D_P  offset: 0x0e06c W
 */
int GPIOSetPullUpDownGPIO4_D( int Pin, enum GPIOPullUpDownState Pull )
{
  uint32_t W32= 0;
/* Set the write enable bits */  
  W32= 3UL << (16+ (2* Pin));
/* The enum GPIOPullUpDownState contains the proper bits already. There is no 
 * verification in this function. 
 * Only the bits are affected, which are selected in the upper part of the register, 
 * by the write enable bits. so it is not necessary to take care of other bits.
 * Simply write. */ 
  W32 |= Pull << (2* Pin);
  *(volatile uint32_t *)( GRF_base+ 0x0e06c)= W32;
  return 0;
}


/*  Read pull-up or pull-down of a GPIO pin*/

/* GPIO1_A
 * 
 * Register PMUGRF_GPIO1A_P offset: 0x00050 W
 */
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO1_A( int Pin )
{
  uint32_t W32;
  W32= *(volatile uint32_t *)( PMUGRF_base+ 0x00050);
/* The enum GPIOPullUpDownState contains the same bit combination as 
 * written to the register for a given pin. So it is necessary only to filter
 * the proper bits, and return them.
*/ 
  return (enum GPIOPullUpDownState )(( W32>>(2* Pin)) &3);
}
/* GPIO1_B
 * 
 * Register PMUGRF_GPIO1B_P   offset: 0x00054 W
 */
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO1_B( int Pin )
{
  uint32_t W32;
  W32= *(volatile uint32_t *)( PMUGRF_base+ 0x00054);
/* The enum GPIOPullUpDownState contains the same bit combination as 
 * written to the register for a given pin. So it is necessary only to filter
 * the proper bits, and return them.
*/ 
  return (enum GPIOPullUpDownState )(( W32>>(2* Pin)) &3);
}
/* GPIO1_C
 * 
 * Register PMUGRF_GPIO1C_P  offset: 0x00058 W
 */
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO1_C( int Pin )
{
  uint32_t W32;
  W32= *(volatile uint32_t *)( PMUGRF_base+ 0x00058);
/* The enum GPIOPullUpDownState contains the same bit combination as 
 * written to the register for a given pin. So it is necessary only to filter
 * the proper bits, and return them.
*/ 
  return (enum GPIOPullUpDownState )(( W32>>(2* Pin)) &3);
}
/* GPIO3_D
 * 
 * Register GRF_GPIO3D_P  offset: 0x0e05c W
 */
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO3_D( int Pin )
{
  uint32_t W32= 0;
  W32= *(volatile uint32_t *)( GRF_base+ 0x0e05c);
/* The enum GPIOPullUpDownState contains the same bit combination as 
 * written to the register for a given pin. So it is necessary only to filter
 * the proper bits, and return them.
*/ 
  return (enum GPIOPullUpDownState )(( W32>>(2* Pin)) &3);
}
/* GPIO4_C
 * 
 * Register GRF_GPIO4C_P  offset: 0x0e068 W
 */
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO4_C( int Pin )
{
  uint32_t W32;
  W32= *(volatile uint32_t *)( GRF_base+ 0x0e068);
/* The enum GPIOPullUpDownState contains the same bit combination as 
 * written to the register for a given pin. So it is necessary only to filter
 * the proper bits, and return them.
*/ 
  return (enum GPIOPullUpDownState )(( W32>>(2* Pin)) &3);
}
/* GPIO4_D
 * 
 * Register GRF_GPIO4D_P  offset: 0x0e06c W
 */
enum GPIOPullUpDownState GPIOGetPullUpDownGPIO4_D( int Pin )
{
  uint32_t W32;
  W32= *(volatile uint32_t *)( GRF_base+ 0x0e06c);
/* The enum GPIOPullUpDownState contains the same bit combination as 
 * written to the register for a given pin. So it is necessary only to filter
 * the proper bits, and return them.
*/ 
  return (enum GPIOPullUpDownState )(( W32>>(2* Pin)) &3);
}

