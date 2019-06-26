# Colour sorting robot

This repository contains all the information (source code, schematics and information) about the colour sorting robot.
The colour sorting robot uses the STM32F103C8T6 microcontroller with the real time operating system `FreeRTOS`. 


The goal is to sort coloured balls positioned at temporarry placeholders and move them to defined final placeholder positions. This should be done autonomously.
![Could not load image](https://i.ibb.co/vJ0bNQF/robot.png "Schematic view of the issue ")

# Table of Contents
1. [Dependencies](#Dependencies)
2. [Hierarchy](#Hierarchy)
3. [Third Example](#third-example)

## Dependencies

* `st-flash`:  write .bin executables to the microcontroller
* `arm-none-eabi`: compile and debug code (do **NOT** use the x86 `gdb` variant. Use `gdb-multiarch` or `arm-none-eabi-gdb` instead)
* `openocd`: setup session between the microcontroller and `gdb`

## Hierarchy
```
boot_s
|_ STM32F103C8T6_boot.S        (startup file)

device_headers
|_ cmsis_gcc.h
|_ core_cm0.h
|_ core_cm3.h
|_ core_cmFunc.h
|_ core_cmInstr.h
|_ core_cmSimd.h
|_ stm32f103xb.h
|_ stm32f1xx.h
|_ system_stm32f1xx.h

ld
|_ STM32F103C8T6.ld            (linker script)

vector_tables
|_ STM32F103C8T6_vt.S          (vector table)

src
|_ main.c
|_ uart
    |_ uart.h
    |_ uart.c
|_ i2c
    |_ i2c.h
    |_ i2c.c


sim
|_ index.html
|_ style.css

FreeRTOSConfig.h
Makefile 
README.md
```

