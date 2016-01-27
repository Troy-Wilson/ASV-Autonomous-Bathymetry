# -*- coding: utf-8 -*-

import serial
import time

# control motors though 1 USB connection, second motor daisy chained on
# use Pololu protocol (see page 60 of pololu simple motor controller manual

class MotorContDaisy():
    def __init__(self,port, portID='\x01', starID='\x02', timeout=1):
        self.ser = serial.Serial(port=port, timeout = timeout)
        self.starID = starID
        self.portID = portID
        self.ser.flush()

        # #check port motor
        # self.ser.write(chr(0xAA)+self.portID+ chr(0x21)+chr(0x00))
        # response = self.ser.read(3) #2 is the number of bytes
        # print len(response)
        # decResponse =  ord(response[0])+(ord(response[1])<<8)
        # print "port response" , decResponse
        # if decResponse >= 1:
        #     self.ser.write(chr(0xAA)+ self.portID+ chr(0x03)) #exit safe start
        # #check star motor
        # self.ser.write(chr(0xAA)+self.starID+ chr(0x21)+chr(0x00))
        # response = self.ser.read(3) #2 is the number of bytes
        # print len(response)
        # decResponse =  ord(response[0])+(ord(response[1])<<8)
        # print "star response", decResponse
        # if decResponse >= 1:
        #     self.ser.write(chr(0xAA)+ self.starID+ chr(0x03)) #exit safe start

    def throttle(self,speed,motorID):
        self.ser.write(chr(0xAA)+ motorID+ chr(0x03)) #hack, rather than trying to read errors just clear
        if speed>100:
            speed = 100
        elif speed<-100:
            speed = -100
        if speed >= 0:
            direction = '\x05'
        elif speed <0:
            speed *= -1
            direction = '\x06'
        integer = int(speed)
        remainder = int((speed - integer)*100)
        self.ser.write(chr(0xAA)+ motorID+ direction+chr(remainder)+chr(integer))

    def throttlePort(self,speed):
        self.throttle(speed, self.portID)

    def throttleStar(self,speed):
        self.throttle(speed, self.starID)

    def close(self):
        self.throttlePort(0.0)
        self.throttleStar(0.0)
        self.ser.close()

if __name__ == '__main__':
    portID = '\x01'
    starID = '\x02'
    mc = MotorContDaisy('/dev/ttyUSB8',portID,starID)

    for i in range(0,200,1):
        if i < 100:        
            mc.throttleStar(i)
            mc.throttlePort(i)
        else:
            mc.throttleStar(200-i)
            mc.throttlePort(200-i)
        print i
        time.sleep(0.1)
 
    for i in range(0,200,1):
        if i < 100:        
            mc.throttleStar(-i)
            mc.throttlePort(-i)
        else:
            mc.throttleStar(-200+i)
            mc.throttlePort(-200+i)
        print i
        time.sleep(0.1) 
   
    mc.close()
