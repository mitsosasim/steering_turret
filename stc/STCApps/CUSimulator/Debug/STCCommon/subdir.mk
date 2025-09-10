################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alma/STCApps/STCCommon/STCConfig.c \
/home/alma/STCApps/STCCommon/STCProtocol.c \
/home/alma/STCApps/STCCommon/STCShData.c 

C_DEPS += \
./STCCommon/STCConfig.d \
./STCCommon/STCProtocol.d \
./STCCommon/STCShData.d 

OBJS += \
./STCCommon/STCConfig.o \
./STCCommon/STCProtocol.o \
./STCCommon/STCShData.o 


# Each subdirectory must supply rules for building sources it contributes
STCCommon/STCConfig.o: /home/alma/STCApps/STCCommon/STCConfig.c STCCommon/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

STCCommon/STCProtocol.o: /home/alma/STCApps/STCCommon/STCProtocol.c STCCommon/subdir.mk
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
	-$(RM) ./STCCommon/STCConfig.d ./STCCommon/STCConfig.o ./STCCommon/STCProtocol.d ./STCCommon/STCProtocol.o ./STCCommon/STCShData.d ./STCCommon/STCShData.o

.PHONY: clean-STCCommon

