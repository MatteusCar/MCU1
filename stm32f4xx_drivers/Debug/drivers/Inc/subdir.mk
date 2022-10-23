################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Inc/stm32f407xx_i2c_driver.c \
../drivers/Inc/stm32f4xx_i2c_driver.c 

OBJS += \
./drivers/Inc/stm32f407xx_i2c_driver.o \
./drivers/Inc/stm32f4xx_i2c_driver.o 

C_DEPS += \
./drivers/Inc/stm32f407xx_i2c_driver.d \
./drivers/Inc/stm32f4xx_i2c_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Inc/%.o drivers/Inc/%.su: ../drivers/Inc/%.c drivers/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Teedy/STM32CubeIDE/workspace_1.10.1/MCU1-20220729T024426Z-001.zip_expanded/MCU1/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Inc

clean-drivers-2f-Inc:
	-$(RM) ./drivers/Inc/stm32f407xx_i2c_driver.d ./drivers/Inc/stm32f407xx_i2c_driver.o ./drivers/Inc/stm32f407xx_i2c_driver.su ./drivers/Inc/stm32f4xx_i2c_driver.d ./drivers/Inc/stm32f4xx_i2c_driver.o ./drivers/Inc/stm32f4xx_i2c_driver.su

.PHONY: clean-drivers-2f-Inc

