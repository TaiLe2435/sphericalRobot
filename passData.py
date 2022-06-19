import serial

arduinoData = serial.Serial('COM14', 9600)

while True:
    cmd = input('Please Enter Your Command: ')
    cmd = cmd + '\r'
    arduinoData.write(cmd.encode())