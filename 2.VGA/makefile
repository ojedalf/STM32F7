#---------------------------------------------------------------------------------------------
# Makefile
# --------------------------------------------------------------------------------------------
# Author: Fernando Ojeda L.                                                           Dic-2019
#---------------------------------------------------------------------------------------------
# Description:
#---------------------------------------------------------------------------------------------
# This makefile contains the targets needed to build the source code and to load the pogram 
# code in the STM32F7 by establishing an OpenOCD connection with Telnet and GDB.
# 
#---------------------------------------------------------------------------------------------
# Installation Settings:
#---------------------------------------------------------------------------------------------
#  
#  1. Install the ARM cross-compiler from its official site, which includes:
#     GCC
#     Binutils (GNU linker, GNU objcopy, GNU objump)
#     GDB debugger
#
#  2. Install the ST-Link drivers from ST site for the discovery board
#
#  3. Install Cygwin along the following packages:
#     autobuild
#     autoconf
#     autoconf-archive
#     automake
#     git
#     gcc-core
#     libtool
#     libusb1.0
#     libusb1.0-devel
#     make
#     pkg-config
#     usbutils
#     inetutils   // Telnet
#
#  4. Create the following directory:
#     mkdir -p /build; cd /build
#
#  5. Download OpenOCD source code by cloning from the official repository:
#     git clone https://git.code.sf.net/p/openocd/code openocd
#
#  6. cd openocd

#  7. It is very important to apply all needed patches to avoid errors when building the openocd
#     source code, by the moment install the patch openocd-for-cygwin.patch from:
# 	  https://mindchasers.com/static/patches/openocd-for-cygwin.patch
#
#  8. Apply the patch:
#     git am < openocd-for-cygwin.patch
#
#  9. If the patch fails with rebase error, type:
#     git am --abort
#
# 10. If the patch fails to apply, type:
#     git apply --reject --whitespace=fix mychanges.patch 
#
# 11. After applying the patch succesfully, type:
#     cd /build/openocd           // change directory
#     ./bootstrap                 // when building from the git repository
#     mkdir build                 // Create directory
#     cd build                    // Change directory
#     ../configure --enable-ftdi  // generates the Makefiles required to build OpenOCD
#     make                        // will build OpenOCD and place the final executable in './src/'
#     make install                // places all of the files in the required location
#
# 12. Confirm that openocd is installed by typing:
#     which openocd 
#
#
#---------------------------------------------------------------------------------------------
# Commands:
#---------------------------------------------------------------------------------------------
#
# --> make build            | Will clean all previous output files and compile the source code
#
# --> make run              | Will load the program and establish a gdb connection with the STM32
#
# --> arm-none-eabi-gcc 
# -c                        | Compile but do not link, output is an object file .o extension
# -o file                   | Place output in file, if not defined it is placed in a.exe
# -g                        | adds the debugginhg information to the object file
# -S                        | Generates assembler code
# -Xlinker -Map=output.map  | creates map file from linker 
#
# --> arm-none-eabi-gdb
# -tui                      |  text based user interface, shows source file and gdb commands
#
# --> exec 1> /dev/pty1     | Display commands in different terminal, "ptyn" represent the 
#                             terminal id which is get by the "tty" command.
#
# --> monitor               | you can send commands to openocd through gdb just like you can 
#                             through a telnet connection. ex.- 
#					          monitor reset halt
#
#---------------------------------------------------------------------------------------------
# Programming steps:
#---------------------------------------------------------------------------------------------
# --> make run
#
# 1. Initiate an OpenOCD server with the .cfg file in order to communicate with the ST-Link 
#
# 2. Initiate a Telnet Session on Port:4444 in order to communicate with the OpenOCD server and
#    to "load" the program code in the STM32 Flash/RAM memory.
#
# 3. Initiate a GDB client session on Port:3333 in order to communicate with OpenOCD server and 
#    to "debug" the program on the STM32 (On-chip debugging)
#
#   
#            |Client:    |               |Server:|    |Debug Adapter:|    |UC:  |
#    .hex--->|Telnet:4444|----TCP/IP---->|OpenOCD|--->|ST-Link       |--->|STM32|
#                                            ^
#            |Client: |                      |
#    .elf--->|GDB:3333|----TCP/IP-------------  
#
#
#---------------------------------------------------------------------------------------------
# Notes:
#---------------------------------------------------------------------------------------------
#
# 1. If the .sh script fails, run the following command in terminal:
#    sed -i 's/\r$//' myscript.sh
#    this will remove the windows style ending lines which complains with linux style
#
# 2. Telnet is disabled by default in windows 10
#
#---------------------------------------------------------------------------------------------



#-----------------------------------------------------------------------------------
# Constants and variables
#-----------------------------------------------------------------------------------
 # Compiler to use
CC = arm-none-eabi-gcc  	

# Architecture
CPU = cortex-m7

# Directory for header files           
INCLUDES = -I ./                                                     

# List of objects to build
OBJS =  startup_simple.o \
		main.o \
		system_init.o \
		ltdc.o

#-----------------------------------------------------------------------------------
# Targets:
#-----------------------------------------------------------------------------------

# Build the program
build:  clean compile
		
#-----------------------------------------------------------------------------------
# Compilation tree dependencies
#-----------------------------------------------------------------------------------
# The main.elf depends on all object files (compile and link all objects)
# Each Object file depends on its respective .c file (compile each without linking)
# Each .c file depends on several include files
#-----------------------------------------------------------------------------------
compile: ${OBJS}	
	${CC} -T simple.ld -o main.elf ${OBJS} -nostartfiles -Xlinker -Map=output.map
	
	@echo "** Generating hex file **"
	arm-none-eabi-objcopy -O ihex main.elf main.hex
	
startup_simple.o: startup_simple.c
	@echo "** Building **"
	arm-none-eabi-gcc -O0 -c -g -mcpu=${CPU} -mthumb startup_simple.c -I ./ 
	
main.o: main.c
	arm-none-eabi-gcc -O0 -c -g -mcpu=${CPU} -mthumb main.c -I ./ 
   
system_init.o: system_init.c
	arm-none-eabi-gcc -O0 -c -g -mcpu=${CPU} -mthumb system_init.c -I ./ 

ltdc.o: ltdc.c
	arm-none-eabi-gcc -O0 -c -g -mcpu=${CPU} -mthumb ltdc.c -I ./    
   
#-----------------------------------------------------------------------------------


run:
	# Start OpenOCD server in new terminal
	mintty -h a -D -e openocd -f /usr/local/share/openocd/scripts/board/stm32f7discovery.cfg  
	sleep 2
	# Start Telnet session on port:4444	
	./telnet.sh
	# Start a GDB client on port:3333
	arm-none-eabi-gdb --eval-command="target remote localhost:3333" main.elf
	

	
run_gdb:
	arm-none-eabi-gdb -tui --eval-command="target remote localhost:3333" main.elf 
		

# Clean all compilation output files
clean:
	@echo "** Cleaning **"
	rm -rf *.o
	rm -rf *.exe
	rm -rf *.elf
	rm -rf *.txt
	rm -rf *.map
	rm -rf *.lst
	rm -rf *.bin
	rm -rf *.hex
		
hex:
	arm-none-eabi-objcopy -O ihex main.elf main.hex

# Dump file in assembly
dump_asm:
	objdump  -j .text -S main.o > my_file.lst

# Dump complete symbols table
read_elf:
	readelf --syms main.elf > main.txt

# Display memory sections	
dump_elf:
	arm-none-eabi-objdump -h main.elf

# Display symbols table
symbols:
	arm-none-eabi-nm --numeric-sort main.elf	
	
gdb_start:
	arm-none-eabi-gdb main.elf
