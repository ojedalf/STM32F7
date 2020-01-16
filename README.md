# STM32F7

----------------------------------------------------------------------------------
1.Led
----------------------------------------------------------------------------------
This project folder presents all the configuration needed from scratch in order to
program and debug the STM32F7 Discovery Kit in batch mode. Telnet and GDB clients 
are set through a TCP/IP connection in order to communicate with an OpenOCD server. 

A linker file is made from scratch in order to match the STM32F7 memory map and 
to load the source code and interrupt vector table in ROM. 

An startup code was implemented in order to copy  the .data and .bss sections from 
ROM to RAM by using symbols from the linker file. The interrups were also defined 
manually and allocated in ROM by ussing GCC compiler attributes.

A top level makefile was made to facilitate the building process by concentrating all
.c dependencies into a target "make build". Also, the source code loading process to 
the STM32F7 ROM and the debugging process connection were both automated by defining a 
target "make run".

The main file contains a simple blinky led for the STM32F7 Discovery board that uses
the systick interrupt and the configuration of registers by using Memory Access Macros.
