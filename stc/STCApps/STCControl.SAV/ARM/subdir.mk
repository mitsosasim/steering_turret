################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STCControl.c \
../STFunctions.c \
../ThCANOperator.c \
../ThListener.c \
../ThSafety.c \
../ThTalker.c 

C_DEPS += \
./STCControl.d \
./STFunctions.d \
./ThCANOperator.d \
./ThListener.d \
./ThSafety.d \
./ThTalker.d 

OBJS += \
./STCControl.o \
./STFunctions.o \
./ThCANOperator.o \
./ThListener.o \
./ThSafety.o \
./ThTalker.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	aarch64-linux-gnu-gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/json/json-c-master -O3 -Wno-unused-result -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./STCControl.d ./STCControl.o ./STFunctions.d ./STFunctions.o ./ThCANOperator.d ./ThCANOperator.o ./ThListener.d ./ThListener.o ./ThSafety.d ./ThSafety.o ./ThTalker.d ./ThTalker.o

.PHONY: clean--2e-

