################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/014i2c_disciple_tx_string.c 

OBJS += \
./Src/014i2c_disciple_tx_string.o 

C_DEPS += \
./Src/014i2c_disciple_tx_string.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Teedy/STM32CubeIDE/workspace_1.10.1/MCU1-20220729T024426Z-001.zip_expanded/MCU1/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/014i2c_disciple_tx_string.d ./Src/014i2c_disciple_tx_string.o ./Src/014i2c_disciple_tx_string.su

.PHONY: clean-Src

