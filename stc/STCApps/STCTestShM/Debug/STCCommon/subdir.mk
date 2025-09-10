################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alma/STCApps/STCCommon/STCConfig.c \
/home/alma/STCApps/STCCommon/STCErrors.c \
/home/alma/STCApps/STCCommon/STCShData.c 

C_DEPS += \
./STCCommon/STCConfig.d \
./STCCommon/STCErrors.d \
./STCCommon/STCShData.d 

OBJS += \
./STCCommon/STCConfig.o \
./STCCommon/STCErrors.o \
./STCCommon/STCShData.o 


# Each subdirectory must supply rules for building sources it contributes
STCCommon/STCConfig.o: /home/alma/STCApps/STCCommon/STCConfig.c STCCommon/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

STCCommon/STCErrors.o: /home/alma/STCApps/STCCommon/STCErrors.c STCCommon/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

STCCommon/STCShData.o: /home/alma/STCApps/STCCommon/STCShData.c STCCommon/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-STCCommon

clean-STCCommon:
	-$(RM) ./STCCommon/STCConfig.d ./STCCommon/STCConfig.o ./STCCommon/STCErrors.d ./STCCommon/STCErrors.o ./STCCommon/STCShData.d ./STCCommon/STCShData.o

.PHONY: clean-STCCommon

