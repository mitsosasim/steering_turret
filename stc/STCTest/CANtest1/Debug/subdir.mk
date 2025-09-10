################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../canfilter.c \
../canreceive.c \
../cantransmit.c 

C_DEPS += \
./canfilter.d \
./canreceive.d \
./cantransmit.d 

OBJS += \
./canfilter.o \
./canreceive.o \
./cantransmit.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./canfilter.d ./canfilter.o ./canreceive.d ./canreceive.o ./cantransmit.d ./cantransmit.o

.PHONY: clean--2e-

