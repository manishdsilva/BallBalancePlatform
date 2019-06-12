import serial
import time


s = p = serial.Serial("COM4",9600) #Establish Serial Communication
print(s.portstr)    #port
#print(dir(s))
s.baudrate=9600
p.baudrate=9600
#s.write('1')
time.sleep(5)
data = "060070"
cha = "120110"
da = "090090"
#print(move_ahead)


#sends data to Arduino which interprets the value and moves the arm

s.write(data.encode())
s.write(cha.encode())
#t = p.readline()
s.write(da.encode())

