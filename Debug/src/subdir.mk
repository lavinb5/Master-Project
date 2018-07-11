################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/PathPlanner.cpp \
../src/Writer.cpp \
../src/rpirobot_serial4.cpp 

OBJS += \
./src/PathPlanner.o \
./src/Writer.o \
./src/rpirobot_serial4.o 

CPP_DEPS += \
./src/PathPlanner.d \
./src/Writer.d \
./src/rpirobot_serial4.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


