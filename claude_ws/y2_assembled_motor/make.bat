@echo off
REM ========================================
REM Motor Control Make Script
REM ========================================

cd /d E:\motor_control

set GCC="C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin\arm-none-eabi-gcc.exe"
set OBJCOPY="C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin\arm-none-eabi-objcopy.exe"
set SIZE="C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin\arm-none-eabi-size.exe"

set FLAGS=-mcpu=cortex-m3 -mthumb -Wall -fdata-sections -ffunction-sections -Og -g -gdwarf-2
set INC=-ICore\Inc -IDrivers\STM32F1xx_HAL_Driver\Inc -IDrivers\STM32F1xx_HAL_Driver\Inc\Legacy -IDrivers\CMSIS\Device\ST\STM32F1xx\Include -IDrivers\CMSIS\Include
set DEF=-DUSE_HAL_DRIVER -DSTM32F103xB

if "%1"=="clean" goto clean
if "%1"=="rebuild" goto rebuild
goto build

:clean
echo ===== Cleaning =====
del /q build\*.o 2>nul
del /q build\*.elf 2>nul
del /q build\*.hex 2>nul
del /q build\*.bin 2>nul
del /q build\*.map 2>nul
echo Clean complete!
echo.
pause
goto end

:rebuild
echo ===== Rebuild =====
call :clean
echo.

:build
echo ===== Building Motor Control =====
echo.

if not exist build mkdir build

echo Compiling Core Src...
%GCC% -c %FLAGS% %INC% %DEF% -o build\main.o Core\Src\main.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\gpio.o Core\Src\gpio.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\adc.o Core\Src\adc.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\tim.o Core\Src\tim.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\usart.o Core\Src\usart.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_it.o Core\Src\stm32f1xx_it.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_msp.o Core\Src\stm32f1xx_hal_msp.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\system_stm32f1xx.o Core\Src\system_stm32f1xx.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\sysmem.o Core\Src\sysmem.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\syscalls.o Core\Src\syscalls.c

echo Compiling HAL Driver...
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_gpio.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_gpio.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_adc.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_adc.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_adc_ex.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_adc_ex.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_rcc.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_rcc.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_rcc_ex.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_rcc_ex.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_dma.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_dma.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_cortex.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_cortex.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_pwr.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_pwr.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_flash.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_flash.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_flash_ex.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_flash_ex.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_tim.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_tim.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_tim_ex.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_tim_ex.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_uart.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_uart.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_gpio_ex.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_gpio_ex.c
%GCC% -c %FLAGS% %INC% %DEF% -o build\stm32f1xx_hal_exti.o Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_exti.c

echo Compiling Startup...
%GCC% -c %FLAGS% %INC% %DEF% -o build\startup_stm32f103xb.o startup_stm32f103xb.s

echo Linking...
%GCC% -mcpu=cortex-m3 -mthumb -specs=nano.specs -T STM32F103XX_FLASH.ld -Wl,-Map=build\Motor_Driver.map,--cref -Wl,--gc-sections build\*.o -o build\Motor_Driver.elf

echo Generating HEX and BIN...
%OBJCOPY% -O ihex build\Motor_Driver.elf build\Motor_Driver.hex
%OBJCOPY% -O binary -S build\Motor_Driver.elf build\Motor_Driver.bin

echo.
echo ===== Build Complete =====
%SIZE% build\Motor_Driver.elf
echo.
dir build\*.elf build\*.hex build\*.bin
echo.
echo Press any key to exit...
pause >nul

:end
