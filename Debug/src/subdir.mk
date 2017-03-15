################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BlackCore.cpp \
../src/BlackI2C.cpp \
../src/BlackUART.cpp \
../src/Profiler.cpp 

OBJS += \
./src/BlackCore.o \
./src/BlackI2C.o \
./src/BlackUART.o \
./src/Profiler.o 

CPP_DEPS += \
./src/BlackCore.d \
./src/BlackI2C.d \
./src/BlackUART.d \
./src/Profiler.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/joao/workspace/helloBBB/BlackLib -I/home/joao/workspace/v3_0 -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


