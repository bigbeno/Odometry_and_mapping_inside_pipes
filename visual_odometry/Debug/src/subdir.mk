################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/appearanceOdometryEstimator.cpp \
../src/circleAnnotator.cpp \
../src/circleOdometryEstimator.cpp \
../src/circleTracker.cpp \
../src/visual_odometry.cpp 

OBJS += \
./src/appearanceOdometryEstimator.o \
./src/circleAnnotator.o \
./src/circleOdometryEstimator.o \
./src/circleTracker.o \
./src/visual_odometry.o 

CPP_DEPS += \
./src/appearanceOdometryEstimator.d \
./src/circleAnnotator.d \
./src/circleOdometryEstimator.d \
./src/circleTracker.d \
./src/visual_odometry.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -std=c++0x -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


