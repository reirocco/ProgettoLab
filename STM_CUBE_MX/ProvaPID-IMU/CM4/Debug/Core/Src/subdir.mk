################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/PID-C1.c \
../Core/Src/PID-C2.c \
../Core/Src/PID-C3.c \
../Core/Src/PID.c \
../Core/Src/PID_Pitch.c \
../Core/Src/PID_Roll.c \
../Core/Src/PID_Yaw.c \
../Core/Src/PWM_Motor1.c \
../Core/Src/PWM_Motor2.c \
../Core/Src/PWM_Motor3.c \
../Core/Src/bno055.c \
../Core/Src/main.c \
../Core/Src/matrice.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c 

OBJS += \
./Core/Src/PID-C1.o \
./Core/Src/PID-C2.o \
./Core/Src/PID-C3.o \
./Core/Src/PID.o \
./Core/Src/PID_Pitch.o \
./Core/Src/PID_Roll.o \
./Core/Src/PID_Yaw.o \
./Core/Src/PWM_Motor1.o \
./Core/Src/PWM_Motor2.o \
./Core/Src/PWM_Motor3.o \
./Core/Src/bno055.o \
./Core/Src/main.o \
./Core/Src/matrice.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o 

C_DEPS += \
./Core/Src/PID-C1.d \
./Core/Src/PID-C2.d \
./Core/Src/PID-C3.d \
./Core/Src/PID.d \
./Core/Src/PID_Pitch.d \
./Core/Src/PID_Roll.d \
./Core/Src/PID_Yaw.d \
./Core/Src/PWM_Motor1.d \
./Core/Src/PWM_Motor2.d \
./Core/Src/PWM_Motor3.d \
./Core/Src/bno055.d \
./Core/Src/main.d \
./Core/Src/matrice.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/PID-C1.d ./Core/Src/PID-C1.o ./Core/Src/PID-C1.su ./Core/Src/PID-C2.d ./Core/Src/PID-C2.o ./Core/Src/PID-C2.su ./Core/Src/PID-C3.d ./Core/Src/PID-C3.o ./Core/Src/PID-C3.su ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/PID_Pitch.d ./Core/Src/PID_Pitch.o ./Core/Src/PID_Pitch.su ./Core/Src/PID_Roll.d ./Core/Src/PID_Roll.o ./Core/Src/PID_Roll.su ./Core/Src/PID_Yaw.d ./Core/Src/PID_Yaw.o ./Core/Src/PID_Yaw.su ./Core/Src/PWM_Motor1.d ./Core/Src/PWM_Motor1.o ./Core/Src/PWM_Motor1.su ./Core/Src/PWM_Motor2.d ./Core/Src/PWM_Motor2.o ./Core/Src/PWM_Motor2.su ./Core/Src/PWM_Motor3.d ./Core/Src/PWM_Motor3.o ./Core/Src/PWM_Motor3.su ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/matrice.d ./Core/Src/matrice.o ./Core/Src/matrice.su ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su

.PHONY: clean-Core-2f-Src

