################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f0discovery_gpio_driver.c 

OBJS += \
./drivers/Src/stm32f0discovery_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32f0discovery_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f0discovery_gpio_driver.o: ../drivers/Src/stm32f0discovery_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F0DISCOVERY -DSTM32 -DSTM32F0 -DDEBUG -DSTM32F051R8Tx -c -I../Inc -I"C:/Users/oguzh/Documents/MCU1-Course/MCU1/f0discovery-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f0discovery_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

