################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/LwIP/src/api/api_lib.c \
../Middlewares/Third_Party/LwIP/src/api/api_msg.c \
../Middlewares/Third_Party/LwIP/src/api/err.c \
../Middlewares/Third_Party/LwIP/src/api/netbuf.c \
../Middlewares/Third_Party/LwIP/src/api/netdb.c \
../Middlewares/Third_Party/LwIP/src/api/netifapi.c \
../Middlewares/Third_Party/LwIP/src/api/sockets.c \
../Middlewares/Third_Party/LwIP/src/api/tcpip.c 

OBJS += \
./Middlewares/Third_Party/LwIP/src/api/api_lib.o \
./Middlewares/Third_Party/LwIP/src/api/api_msg.o \
./Middlewares/Third_Party/LwIP/src/api/err.o \
./Middlewares/Third_Party/LwIP/src/api/netbuf.o \
./Middlewares/Third_Party/LwIP/src/api/netdb.o \
./Middlewares/Third_Party/LwIP/src/api/netifapi.o \
./Middlewares/Third_Party/LwIP/src/api/sockets.o \
./Middlewares/Third_Party/LwIP/src/api/tcpip.o 

C_DEPS += \
./Middlewares/Third_Party/LwIP/src/api/api_lib.d \
./Middlewares/Third_Party/LwIP/src/api/api_msg.d \
./Middlewares/Third_Party/LwIP/src/api/err.d \
./Middlewares/Third_Party/LwIP/src/api/netbuf.d \
./Middlewares/Third_Party/LwIP/src/api/netdb.d \
./Middlewares/Third_Party/LwIP/src/api/netifapi.d \
./Middlewares/Third_Party/LwIP/src/api/sockets.d \
./Middlewares/Third_Party/LwIP/src/api/tcpip.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/LwIP/src/api/%.o: ../Middlewares/Third_Party/LwIP/src/api/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/heap_expr/Eurobot/Inc" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/CMSIS/Include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/apps/httpd"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


