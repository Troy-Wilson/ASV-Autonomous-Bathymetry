# -*- coding: utf-8 -*-

import serial
import time

class MotorCont():
    def __init__(self,port, timeout=1):
        self.ser = serial.Serial(port=port, timeout = timeout)
        self.ser.write(chr(0xAA)) # call auto-baud detector
        self.ser.flush()
        #print self.ser
        #this is a little nasty, should implement better error handling
        #Get errors
        self.ser.write(chr(0xA1)+chr(0x00))
        response = self.ser.read(3) #2 is the number of bytes
        decResponse =  ord(response[0])+(ord(response[1])<<8)
        if decResponse >= 1:
            self.ser.write(chr(0x83)) #exit safe start
        #elif decResponse >1:
            print self.ser.port, 'error code = ', decResponse
    
    def throttle(self,speed):
        self.ser.write(chr(0x83)) #hack, rather than trying to read errors just clear
        if speed>100:
            speed = 100
        elif speed<-100:
            speed = -100
        if speed >= 0:
            direction = '\x85'
        elif speed <0:
            speed *= -1
            direction = '\x86' 
        integer = int(speed)
        remainder = int((speed - integer)*100)
        self.ser.write(direction+chr(remainder)+chr(integer))
        #self.ser.write(direction+chr(0) + chr(speed))
            
    def close(self):
        self.throttle(0.0)
        self.ser.close()

if __name__ == '__main__':        
    mstar = MotorCont(port = '/dev/pololu_starboard')
    mport = MotorCont(port = '/dev/pololu_port')
    #for i in range(0,10):
    #    m1.throttle(i)
        #m2.throttle(i)
    #    print i
    #time.sleep(1)
        
    for i in range(0,200,1):
        if i < 100:        
            mstar.throttle(i)
            mport.throttle(i)
        else:
            mstar.throttle(200-i)
            mport.throttle(200-i)            
        print i
        time.sleep(0.1)
 
    for i in range(0,200,1):
        if i < 100:        
            mstar.throttle(-i)
            mport.throttle(-i)
        else:
            mstar.throttle(-200+i)
            mport.throttle(-200+i)            
        print i
        time.sleep(0.1) 
   
    mstar.close()
    mport.close()
