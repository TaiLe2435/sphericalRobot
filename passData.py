#################################################################
# Tommy Le
# Northern Illinois University, Mechanical Engineering Dept
# 6/19/2022
# This is a basic example showing how serial communication works.
# This code is to be ran with readDatafromPython.ino
#################################################################

import serial      #lib for serial communication

arduinoData = serial.Serial('COM14', 9600)  #initializing port

while True:         #inf loop
    cmd = input('Please Enter Your Command: ') #take user input
    cmd = cmd + '\r'    #add \r to indicate end of string
    arduinoData.write(cmd.encode()) #write to serial port