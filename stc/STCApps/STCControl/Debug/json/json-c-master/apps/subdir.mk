################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alma/STCApps/json/json-c-master/apps/json_parse.c 

C_DEPS += \
./json/json-c-master/apps/json_parse.d 

OBJS += \
./json/json-c-master/apps/json_parse.o 


# Each subdirectory must supply rules for building sources it contributes
json/json-c-master/apps/json_parse.o: /home/alma/STCApps/json/json-c-master/apps/json_parse.c json/json-c-master/apps/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-json-2f-json-2d-c-2d-master-2f-apps

clean-json-2f-json-2d-c-2d-master-2f-apps:
	-$(RM) ./json/json-c-master/apps/json_parse.d ./json/json-c-master/apps/json_parse.o

.PHONY: clean-json-2f-json-2d-c-2d-master-2f-apps

