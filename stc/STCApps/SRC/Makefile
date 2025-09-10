OUT_FILES   :=    libROCKPro64GPIO.a GPIOPinRead GPIOPinWrite GPIOPinSetPull GPIOSpeedTest
HFilesL   :=    ROCKPro64GPIO_Globals.h  ROCKPro64GPIO.h

# This flags are used to create standalone MinGW application under cygwin
#CFLAGS=-mno-cygwin
#
# Large file support
# CFLAGS=-D_FILE_OFFSET_BITS=64 -Wall
# Debuggung
CFLAGS= -g -Wall -pthread -I/usr/local/include 
# Some additional debug messages in coding:
#CFLAGS= -DDebugMessages
LOADLIBES= -L/usr/local/lib -lm -lpthread -lrt libROCKPro64GPIO.a
CC=gcc 

out_files   : $(OUT_FILES)

libROCKPro64GPIO.a	:	ROCKPro64GPIO.o ROCKPro64GPIO_PullUpDown.o ROCKPro64GPIO_LockUnlock.o
		ar ruv libROCKPro64GPIO.a ROCKPro64GPIO.o ROCKPro64GPIO_PullUpDown.o ROCKPro64GPIO_LockUnlock.o
		ranlib libROCKPro64GPIO.a

ROCKPro64GPIO.o	:	$(HFilesL) ROCKPro64GPIO.c 

ROCKPro64GPIO_PullUpDown.o	:	$(HFilesL) ROCKPro64GPIO_PullUpDown.c							

ROCKPro64GPIO_LockUnlock.o	:	$(HFilesL) ROCKPro64GPIO_LockUnlock.c


