################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/003IRQ_led.c 

OBJS += \
./Src/003IRQ_led.o 

C_DEPS += \
./Src/003IRQ_led.d 


# Each subdirectory must supply rules for building sources it contributes
Src/003IRQ_led.o: ../Src/003IRQ_led.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/dev/Embedded-C/My_workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/003IRQ_led.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

