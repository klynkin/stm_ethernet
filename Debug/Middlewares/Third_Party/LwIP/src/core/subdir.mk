################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/LwIP/src/core/def.c \
../Middlewares/Third_Party/LwIP/src/core/dns.c \
../Middlewares/Third_Party/LwIP/src/core/inet_chksum.c \
../Middlewares/Third_Party/LwIP/src/core/init.c \
../Middlewares/Third_Party/LwIP/src/core/ip.c \
../Middlewares/Third_Party/LwIP/src/core/mem.c \
../Middlewares/Third_Party/LwIP/src/core/memp.c \
../Middlewares/Third_Party/LwIP/src/core/netif.c \
../Middlewares/Third_Party/LwIP/src/core/pbuf.c \
../Middlewares/Third_Party/LwIP/src/core/raw.c \
../Middlewares/Third_Party/LwIP/src/core/stats.c \
../Middlewares/Third_Party/LwIP/src/core/sys.c \
../Middlewares/Third_Party/LwIP/src/core/tcp.c \
../Middlewares/Third_Party/LwIP/src/core/tcp_in.c \
../Middlewares/Third_Party/LwIP/src/core/tcp_out.c \
../Middlewares/Third_Party/LwIP/src/core/timeouts.c \
../Middlewares/Third_Party/LwIP/src/core/udp.c 

OBJS += \
./Middlewares/Third_Party/LwIP/src/core/def.o \
./Middlewares/Third_Party/LwIP/src/core/dns.o \
./Middlewares/Third_Party/LwIP/src/core/inet_chksum.o \
./Middlewares/Third_Party/LwIP/src/core/init.o \
./Middlewares/Third_Party/LwIP/src/core/ip.o \
./Middlewares/Third_Party/LwIP/src/core/mem.o \
./Middlewares/Third_Party/LwIP/src/core/memp.o \
./Middlewares/Third_Party/LwIP/src/core/netif.o \
./Middlewares/Third_Party/LwIP/src/core/pbuf.o \
./Middlewares/Third_Party/LwIP/src/core/raw.o \
./Middlewares/Third_Party/LwIP/src/core/stats.o \
./Middlewares/Third_Party/LwIP/src/core/sys.o \
./Middlewares/Third_Party/LwIP/src/core/tcp.o \
./Middlewares/Third_Party/LwIP/src/core/tcp_in.o \
./Middlewares/Third_Party/LwIP/src/core/tcp_out.o \
./Middlewares/Third_Party/LwIP/src/core/timeouts.o \
./Middlewares/Third_Party/LwIP/src/core/udp.o 

C_DEPS += \
./Middlewares/Third_Party/LwIP/src/core/def.d \
./Middlewares/Third_Party/LwIP/src/core/dns.d \
./Middlewares/Third_Party/LwIP/src/core/inet_chksum.d \
./Middlewares/Third_Party/LwIP/src/core/init.d \
./Middlewares/Third_Party/LwIP/src/core/ip.d \
./Middlewares/Third_Party/LwIP/src/core/mem.d \
./Middlewares/Third_Party/LwIP/src/core/memp.d \
./Middlewares/Third_Party/LwIP/src/core/netif.d \
./Middlewares/Third_Party/LwIP/src/core/pbuf.d \
./Middlewares/Third_Party/LwIP/src/core/raw.d \
./Middlewares/Third_Party/LwIP/src/core/stats.d \
./Middlewares/Third_Party/LwIP/src/core/sys.d \
./Middlewares/Third_Party/LwIP/src/core/tcp.d \
./Middlewares/Third_Party/LwIP/src/core/tcp_in.d \
./Middlewares/Third_Party/LwIP/src/core/tcp_out.d \
./Middlewares/Third_Party/LwIP/src/core/timeouts.d \
./Middlewares/Third_Party/LwIP/src/core/udp.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/LwIP/src/core/%.o: ../Middlewares/Third_Party/LwIP/src/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Keil_v5/cube/heap_expr/Eurobot/Inc" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/system" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/posix" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Drivers/CMSIS/Include" -I"C:/Keil_v5/cube/heap_expr/Eurobot/Middlewares/Third_Party/LwIP/src/apps/httpd" -I"C:/Keil_v5/cube/heap_expr/Eurobot/SPL/inc" -I"C:/Keil_v5/cube/heap_expr/Eurobot/SPL/src"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


