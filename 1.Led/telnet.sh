#---------------------------------------------------------------------------------------------
# Makefile
# --------------------------------------------------------------------------------------------
# Author: Fernando Ojeda L.                                                           Dic-2019
#---------------------------------------------------------------------------------------------
# Description:
#---------------------------------------------------------------------------------------------
#  Due to the makefile opens a new terminal automatically to run all programming steps and 
#  stablish a telnet connection, the following script presents a data structure in bash with 
#  all the necessary programming steps through telnet so that they can be run automatically
# 
#---------------------------------------------------------------------------------------------

#!/usr/bin/sh

{
sleep 5
echo reset halt
sleep 3
echo flash write_image erase main.hex 0
sleep 3
echo reset halt
sleep 5
echo exit
} | telnet localhost 4444