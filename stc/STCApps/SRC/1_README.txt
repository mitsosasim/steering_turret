Simple C library to operate GPIO connector on the ROCKPro64 board.

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
Very initial, still in development! Use on your own risk!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

sysfs interface is not used. Only direct access to memory registers. All implemented in user-space, by mmap() and /dev/mem insterface.

Using sysfs accesses in parallel to this library may lead to diverse malfunctions!

High-level functions - the functions intended for calls in user programs - handle collisions by means of mutexes. This may lead to delays in switching pin state, in case of concurrent programs handling GPIO pins.
Low-level functions - the functions which directly access registers in memory - are not protected and are NOT thread-safe. They are not intended for direct usage in user programs.

Currently, only GPIO functionality has been implemented. No e.g. SPI or PWM functions.

-----------------------------------------------------------------------------------------

High-level functions handling GPIO have names GPIOxxxxxxx()
Low-level functions have names GPIOxxxxxxGPIOn_A/B/C/D()

High-level functions accept pin numbers of the 40-pin connector on the ROCKPro64 board. 
Low-level functions have GPIO block number and section in name, and accept pin number in thsi bloc and section. 

Unless othervise specified, functions return int value with status. 0 means success. Better check this return code, to avoid severe malfunctions or mistakes.

-----------------------------------------------------------------------------------------

High-level functions for usage in user programs (include ROCKPro64GPIO.h):

--

int GPIOInit( void)
Must be called before any further functions from this librabry can be used.
Initializes memory blocks, mapping structures and maps access to hardware registers.

--

int GPIOPinInit( int Pin, enum GPIOPinMode Mode)
Prepares a pin Pin for usage as GPIO.
Must be called before any further GPIO operations with the pin are performed.
Mode=GPIOPinModeGPIO must be used for further operate GPIO function of the pin.
Please, remember to call GPIOPinRelease( Pin ), when the pin is not used anymore. This will reset GPIO clock to original state and other functionalities, if possible.

--

int GPIOPinRelease( int Pin)
Returns setup, related to the pin Pin to the original state. It means, the state, before function  GPIOPinInit( Pin, GPIOPinModeGPIO) has been called.
Should be called when the pin Pin is no more needed, for every call of GPIOPinInit( Pin, GPIOPinModeGPIO).

--

int GPIOSetPinDirection( int Pin, enum GPIOPinDirection Dir)
Sets direction for pin Pin. Should be used to properly set direction before writting or reading the pin. 
  Dir=GPIOPinDirectionInput means the pin will be used to sense signals coming to the pin.
  Dir=GPIOPinDirectionOutput means the pin will be used to actively drive state of the connected line.
--

int GPIOSetPinState( int Pin, enum GPIOPinState State )
Sets state of GPIO pin Pin. If the pin has been set to output pin with GPIOSetPinDirection(), this state will be propagated to pin and the connected wire.
If the pin is set to input, the state will be written to the GPIO register, and becomes active, as soon as the pin is set to output.
Possible states:
  State=GPIOPinState0 - voltage around 0 at the pin.
  State=GPIOPinState1 - voltage around 3V at the pin.
  
This function involves mutex operation to prevent race condition. It is worthy to check the return code, as it may contain error message.

--

int GPIOSetPinState_unlocked( int Pin, enum GPIOPinState State )
Same as GPIOSetPinState(), but not protected against race condition. Still the function attepts to preerve state of pins, which it should not affect.
Use with great care!

--

enum GPIOPinState GPIOGetPinState( int Pin )
Returns state of the pin Pin, It means voltage at the pin. In case of output pin, it is the state set by the fuctnion GPIOSetPinState().
Possible return values:
  GPIOPinState0 - voltage around 0 at the pin.
  GPIOPinState1 - voltage around 3V at the pin.
  
--

int GPIOSetPullUpDown( int Pin, enum GPIOPullUpDownState Pull )
Sets pull-up or pull-down for the pin Pin. This setting has influence on return value of GPIOGetPinState(), even if no signal is connected to GPIO.
ATTENTION: some pins do not react on this setting, but seem to have harwired pull-up or pull-down. This needs further clarification.
Possible Pull values:
  Pull=GPIOPullUpDownStateNormal - no active pull-up or pull-down.
  Pull=GPIOPullUpDownStateUp - pin actively pulled to 3V.
  Pull=GPIOPullUpDownStateDown - pin actively pulled to 0V.

--

int GPIODriveStrength( int Pin, enum GPIODriveStrengthState Current )
Set drive strenght of pin Pin. According to TRM document of RK3399, GPIO pins may generate different maximum current. This function sets this maximum current for the pin. All GPIO pins exposed at the 40-pin connector on ROCKPro64 have following possible settings:
  Current= GPIODriveStrengthState3mA
  Current= GPIODriveStrengthState6mA
  Current= GPIODriveStrengthState9mA
  Current= GPIODriveStrengthState12mA

--

enum GPIOPullUpDownState GPIOGetPullUpDown( int Pin )
Delivers current pull-up/down setting of the pin Pin. 

-----------------------------------------------------------------------------------------


Example programs using the library:

GPIOPinRead.c 
Set pin to input and read its state.
Call:
  GPIOPinRead Pin-number-in-40-pin-connector
  
--

GPIOPinSetPull.c
Set pul-up/down of a pin.
Call:
  GPIOPinRead Pin-number-in-40-pin-connector u/d/n
    u - set pull-up
    d - set pull-down
    n - set to normal - meither down, nor up.
The pin is not set to input!
  
--

GPIOPinWrite.c
Set pin to output and set its state.
Call:
  GPIOPinWrite Pin-number-in-40-pin-connector 0/1
    0 - Set pin to 0V
    1 - Set pin to 3V

--

GPIOSpeedTest.c
Testing speed:
   Two pins of the 40-pin GPIO connector are connected with a cable.
   One pin is for reading, the second - for writing.
   The program changes state of the writing-pin to 1. Then waits when this state is sensed on
   the reading-pin. When the 1 is read from the reading-pin, then state of the writing-pin is
   changed again to 0, and the program waits untill the 0 is read from the reading-pin.
   The above sequence is repeated several times, and the time for the complete loop is measured.
   This gives good idea of how quickly signals can be generated by GPIO.
 
   The program initially sets both pins to input mode and displays message to connect the pins.
   Then waits for key-press, and starts the loop.
   This is in order to avoid damages by connecting posisbly active pins. 
 



