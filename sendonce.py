import serial

ser = serial.Serial('COM7')

ser.read()
ser.write(b"150.00,132.24,67.12\r\n")
ser.close()

