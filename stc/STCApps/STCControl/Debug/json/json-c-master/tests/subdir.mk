################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alma/STCApps/json/json-c-master/tests/parse_flags.c \
/home/alma/STCApps/json/json-c-master/tests/test1.c \
/home/alma/STCApps/json/json-c-master/tests/test2.c \
/home/alma/STCApps/json/json-c-master/tests/test4.c \
/home/alma/STCApps/json/json-c-master/tests/testReplaceExisting.c \
/home/alma/STCApps/json/json-c-master/tests/test_cast.c \
/home/alma/STCApps/json/json-c-master/tests/test_charcase.c \
/home/alma/STCApps/json/json-c-master/tests/test_compare.c \
/home/alma/STCApps/json/json-c-master/tests/test_deep_copy.c \
/home/alma/STCApps/json/json-c-master/tests/test_double_serializer.c \
/home/alma/STCApps/json/json-c-master/tests/test_float.c \
/home/alma/STCApps/json/json-c-master/tests/test_int_add.c \
/home/alma/STCApps/json/json-c-master/tests/test_int_get.c \
/home/alma/STCApps/json/json-c-master/tests/test_json_patch.c \
/home/alma/STCApps/json/json-c-master/tests/test_json_pointer.c \
/home/alma/STCApps/json/json-c-master/tests/test_locale.c \
/home/alma/STCApps/json/json-c-master/tests/test_null.c \
/home/alma/STCApps/json/json-c-master/tests/test_object_iterator.c \
/home/alma/STCApps/json/json-c-master/tests/test_parse.c \
/home/alma/STCApps/json/json-c-master/tests/test_parse_int64.c \
/home/alma/STCApps/json/json-c-master/tests/test_printbuf.c \
/home/alma/STCApps/json/json-c-master/tests/test_set_serializer.c \
/home/alma/STCApps/json/json-c-master/tests/test_set_value.c \
/home/alma/STCApps/json/json-c-master/tests/test_strerror.c \
/home/alma/STCApps/json/json-c-master/tests/test_util_file.c \
/home/alma/STCApps/json/json-c-master/tests/test_visit.c 

C_DEPS += \
./json/json-c-master/tests/parse_flags.d \
./json/json-c-master/tests/test1.d \
./json/json-c-master/tests/test2.d \
./json/json-c-master/tests/test4.d \
./json/json-c-master/tests/testReplaceExisting.d \
./json/json-c-master/tests/test_cast.d \
./json/json-c-master/tests/test_charcase.d \
./json/json-c-master/tests/test_compare.d \
./json/json-c-master/tests/test_deep_copy.d \
./json/json-c-master/tests/test_double_serializer.d \
./json/json-c-master/tests/test_float.d \
./json/json-c-master/tests/test_int_add.d \
./json/json-c-master/tests/test_int_get.d \
./json/json-c-master/tests/test_json_patch.d \
./json/json-c-master/tests/test_json_pointer.d \
./json/json-c-master/tests/test_locale.d \
./json/json-c-master/tests/test_null.d \
./json/json-c-master/tests/test_object_iterator.d \
./json/json-c-master/tests/test_parse.d \
./json/json-c-master/tests/test_parse_int64.d \
./json/json-c-master/tests/test_printbuf.d \
./json/json-c-master/tests/test_set_serializer.d \
./json/json-c-master/tests/test_set_value.d \
./json/json-c-master/tests/test_strerror.d \
./json/json-c-master/tests/test_util_file.d \
./json/json-c-master/tests/test_visit.d 

OBJS += \
./json/json-c-master/tests/parse_flags.o \
./json/json-c-master/tests/test1.o \
./json/json-c-master/tests/test2.o \
./json/json-c-master/tests/test4.o \
./json/json-c-master/tests/testReplaceExisting.o \
./json/json-c-master/tests/test_cast.o \
./json/json-c-master/tests/test_charcase.o \
./json/json-c-master/tests/test_compare.o \
./json/json-c-master/tests/test_deep_copy.o \
./json/json-c-master/tests/test_double_serializer.o \
./json/json-c-master/tests/test_float.o \
./json/json-c-master/tests/test_int_add.o \
./json/json-c-master/tests/test_int_get.o \
./json/json-c-master/tests/test_json_patch.o \
./json/json-c-master/tests/test_json_pointer.o \
./json/json-c-master/tests/test_locale.o \
./json/json-c-master/tests/test_null.o \
./json/json-c-master/tests/test_object_iterator.o \
./json/json-c-master/tests/test_parse.o \
./json/json-c-master/tests/test_parse_int64.o \
./json/json-c-master/tests/test_printbuf.o \
./json/json-c-master/tests/test_set_serializer.o \
./json/json-c-master/tests/test_set_value.o \
./json/json-c-master/tests/test_strerror.o \
./json/json-c-master/tests/test_util_file.o \
./json/json-c-master/tests/test_visit.o 


# Each subdirectory must supply rules for building sources it contributes
json/json-c-master/tests/parse_flags.o: /home/alma/STCApps/json/json-c-master/tests/parse_flags.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test1.o: /home/alma/STCApps/json/json-c-master/tests/test1.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test2.o: /home/alma/STCApps/json/json-c-master/tests/test2.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test4.o: /home/alma/STCApps/json/json-c-master/tests/test4.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/testReplaceExisting.o: /home/alma/STCApps/json/json-c-master/tests/testReplaceExisting.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_cast.o: /home/alma/STCApps/json/json-c-master/tests/test_cast.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_charcase.o: /home/alma/STCApps/json/json-c-master/tests/test_charcase.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_compare.o: /home/alma/STCApps/json/json-c-master/tests/test_compare.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_deep_copy.o: /home/alma/STCApps/json/json-c-master/tests/test_deep_copy.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_double_serializer.o: /home/alma/STCApps/json/json-c-master/tests/test_double_serializer.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_float.o: /home/alma/STCApps/json/json-c-master/tests/test_float.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_int_add.o: /home/alma/STCApps/json/json-c-master/tests/test_int_add.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_int_get.o: /home/alma/STCApps/json/json-c-master/tests/test_int_get.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_json_patch.o: /home/alma/STCApps/json/json-c-master/tests/test_json_patch.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_json_pointer.o: /home/alma/STCApps/json/json-c-master/tests/test_json_pointer.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_locale.o: /home/alma/STCApps/json/json-c-master/tests/test_locale.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_null.o: /home/alma/STCApps/json/json-c-master/tests/test_null.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_object_iterator.o: /home/alma/STCApps/json/json-c-master/tests/test_object_iterator.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_parse.o: /home/alma/STCApps/json/json-c-master/tests/test_parse.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_parse_int64.o: /home/alma/STCApps/json/json-c-master/tests/test_parse_int64.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_printbuf.o: /home/alma/STCApps/json/json-c-master/tests/test_printbuf.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_set_serializer.o: /home/alma/STCApps/json/json-c-master/tests/test_set_serializer.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_set_value.o: /home/alma/STCApps/json/json-c-master/tests/test_set_value.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_strerror.o: /home/alma/STCApps/json/json-c-master/tests/test_strerror.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_util_file.o: /home/alma/STCApps/json/json-c-master/tests/test_util_file.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

json/json-c-master/tests/test_visit.o: /home/alma/STCApps/json/json-c-master/tests/test_visit.c json/json-c-master/tests/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/alma/STCApps/STCCommon -I/home/alma/STCApps/STCCommon/json-master -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-json-2f-json-2d-c-2d-master-2f-tests

clean-json-2f-json-2d-c-2d-master-2f-tests:
	-$(RM) ./json/json-c-master/tests/parse_flags.d ./json/json-c-master/tests/parse_flags.o ./json/json-c-master/tests/test1.d ./json/json-c-master/tests/test1.o ./json/json-c-master/tests/test2.d ./json/json-c-master/tests/test2.o ./json/json-c-master/tests/test4.d ./json/json-c-master/tests/test4.o ./json/json-c-master/tests/testReplaceExisting.d ./json/json-c-master/tests/testReplaceExisting.o ./json/json-c-master/tests/test_cast.d ./json/json-c-master/tests/test_cast.o ./json/json-c-master/tests/test_charcase.d ./json/json-c-master/tests/test_charcase.o ./json/json-c-master/tests/test_compare.d ./json/json-c-master/tests/test_compare.o ./json/json-c-master/tests/test_deep_copy.d ./json/json-c-master/tests/test_deep_copy.o ./json/json-c-master/tests/test_double_serializer.d ./json/json-c-master/tests/test_double_serializer.o ./json/json-c-master/tests/test_float.d ./json/json-c-master/tests/test_float.o ./json/json-c-master/tests/test_int_add.d ./json/json-c-master/tests/test_int_add.o ./json/json-c-master/tests/test_int_get.d ./json/json-c-master/tests/test_int_get.o ./json/json-c-master/tests/test_json_patch.d ./json/json-c-master/tests/test_json_patch.o ./json/json-c-master/tests/test_json_pointer.d ./json/json-c-master/tests/test_json_pointer.o ./json/json-c-master/tests/test_locale.d ./json/json-c-master/tests/test_locale.o ./json/json-c-master/tests/test_null.d ./json/json-c-master/tests/test_null.o ./json/json-c-master/tests/test_object_iterator.d ./json/json-c-master/tests/test_object_iterator.o ./json/json-c-master/tests/test_parse.d ./json/json-c-master/tests/test_parse.o ./json/json-c-master/tests/test_parse_int64.d ./json/json-c-master/tests/test_parse_int64.o ./json/json-c-master/tests/test_printbuf.d ./json/json-c-master/tests/test_printbuf.o ./json/json-c-master/tests/test_set_serializer.d ./json/json-c-master/tests/test_set_serializer.o ./json/json-c-master/tests/test_set_value.d ./json/json-c-master/tests/test_set_value.o ./json/json-c-master/tests/test_strerror.d ./json/json-c-master/tests/test_strerror.o ./json/json-c-master/tests/test_util_file.d ./json/json-c-master/tests/test_util_file.o ./json/json-c-master/tests/test_visit.d ./json/json-c-master/tests/test_visit.o

.PHONY: clean-json-2f-json-2d-c-2d-master-2f-tests

