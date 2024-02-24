################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm3232f407xx_GPIO_driver.c \
../drivers/Src/stm32f407xx_I2C_driver.c \
../drivers/Src/stm32f407xx_SPI_driver.c \
../drivers/Src/stm32f407xx_USART_driver.c 

OBJS += \
./drivers/Src/stm3232f407xx_GPIO_driver.o \
./drivers/Src/stm32f407xx_I2C_driver.o \
./drivers/Src/stm32f407xx_SPI_driver.o \
./drivers/Src/stm32f407xx_USART_driver.o 

C_DEPS += \
./drivers/Src/stm3232f407xx_GPIO_driver.d \
./drivers/Src/stm32f407xx_I2C_driver.d \
./drivers/Src/stm32f407xx_SPI_driver.d \
./drivers/Src/stm32f407xx_USART_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/daher/Documents/Embedded-C/instructor/target/stm32f407_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm3232f407xx_GPIO_driver.cyclo ./drivers/Src/stm3232f407xx_GPIO_driver.d ./drivers/Src/stm3232f407xx_GPIO_driver.o ./drivers/Src/stm3232f407xx_GPIO_driver.su ./drivers/Src/stm32f407xx_I2C_driver.cyclo ./drivers/Src/stm32f407xx_I2C_driver.d ./drivers/Src/stm32f407xx_I2C_driver.o ./drivers/Src/stm32f407xx_I2C_driver.su ./drivers/Src/stm32f407xx_SPI_driver.cyclo ./drivers/Src/stm32f407xx_SPI_driver.d ./drivers/Src/stm32f407xx_SPI_driver.o ./drivers/Src/stm32f407xx_SPI_driver.su ./drivers/Src/stm32f407xx_USART_driver.cyclo ./drivers/Src/stm32f407xx_USART_driver.d ./drivers/Src/stm32f407xx_USART_driver.o ./drivers/Src/stm32f407xx_USART_driver.su

.PHONY: clean-drivers-2f-Src

