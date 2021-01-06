################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001led_toggle.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/001led_toggle.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/001led_toggle.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/001led_toggle.o: ../Src/001led_toggle.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F0DISCOVERY -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"C:/Users/oguzh/Documents/MCU1-Course/MCU1/f0discovery-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/001led_toggle.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F0DISCOVERY -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"C:/Users/oguzh/Documents/MCU1-Course/MCU1/f0discovery-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F0DISCOVERY -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"C:/Users/oguzh/Documents/MCU1-Course/MCU1/f0discovery-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

