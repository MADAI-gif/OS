################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/uart/fsl_adapter_usart.c 

OBJS += \
./component/uart/fsl_adapter_usart.o 

C_DEPS += \
./component/uart/fsl_adapter_usart.d 


# Each subdirectory must supply rules for building sources it contributes
component/uart/%.o: ../component/uart/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_LPC55S69JBD100 -DCPU_LPC55S69JBD100_cm33 -DCPU_LPC55S69JBD100_cm33_core0 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSERIAL_PORT_TYPE_UART=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__NEWLIB__ -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/drivers" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/LPC55S69/drivers" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/utilities" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/component/uart" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/component/serial_manager" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/component/lists" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/CMSIS" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/device" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/board" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/source" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/drivers" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/LPC55S69/drivers" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/utilities" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/component/uart" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/component/serial_manager" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/component/lists" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/startup" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/CMSIS" -I"/home/eric/dev/baremetal/nxp-lpc55s69-evk/labos-lpc55s69-evk/LPC55S69_OS.src/device" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -specs=nano.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


