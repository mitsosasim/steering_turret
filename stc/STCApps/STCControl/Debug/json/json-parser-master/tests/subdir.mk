################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alma/STCApps/json/json-parser-master/tests/test.c 

C_DEPS += \
./json/json-parser-master/tests/test.d 

OBJS += \
./json/json-parser-master/tests/test.o 


# Each subdirectory must supply rules for building sources it contributes
json/json-parser-master/tests/test.o: /home/alma/STCApps/json/json-parser-master/tests/test.c json/json-parser-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-json-2f-json-2d-parser-2d-master-2f-tests

clean-json-2f-json-2d-parser-2d-master-2f-tests:
	-$(RM) ./json/json-parser-master/tests/test.d ./json/json-parser-master/tests/test.o

.PHONY: clean-json-2f-json-2d-parser-2d-master-2f-tests

