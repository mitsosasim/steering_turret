################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STCControl.c \
../ThCANOperator.c \
../ThListener.c \
../ThSafety.c \
../ThTalker.c 

C_DEPS += \
./STCControl.d \
./ThCANOperator.d \
./ThListener.d \
./ThSafety.d \
./ThTalker.d 

OBJS += \
./STCControl.o \
./ThCANOperator.o \
./ThListener.o \
./ThSafety.o \
./ThTalker.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./STCControl.d ./STCControl.o ./ThCANOperator.d ./ThCANOperator.o ./ThListener.d ./ThListener.o ./ThSafety.d ./ThSafety.o ./ThTalker.d ./ThTalker.o

.PHONY: clean--2e-

