################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CUSIMReceiver.c \
../CUSim.c \
../CUSimSender.c 

C_DEPS += \
./CUSIMReceiver.d \
./CUSim.d \
./CUSimSender.d 

OBJS += \
./CUSIMReceiver.o \
./CUSim.o \
./CUSimSender.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./CUSIMReceiver.d ./CUSIMReceiver.o ./CUSim.d ./CUSim.o ./CUSimSender.d ./CUSimSender.o

.PHONY: clean--2e-

