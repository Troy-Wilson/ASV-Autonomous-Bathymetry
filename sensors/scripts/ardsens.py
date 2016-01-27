#!/usr/bin/env python
"""
Created on Tue Feb 11 13:54:45 2014

@author: troy
"""

import roslib; roslib.load_manifest('sensors')
import serial, threading
import rospy
from  sensors.msg import Leak, Temp
                     
class ardSens(threading.Thread):
    def __init__(self,port='/dev/ttyACM0', baud=9600, timeout=1, threshold = 300):
        self.ser = serial.Serial(
                                 port,
                                 baudrate = baud,
                                 timeout = timeout,
                                 parity = serial.PARITY_NONE, 
                                 rtscts = False, 
                                 xonxoff = False,
                                 bytesize = serial.EIGHTBITS
                                 )                                 
        self.ser.flush()
        self.pubLeak = rospy.Publisher('leakData', Leak, queue_size = 1)
        self.pubTemp = rospy.Publisher('Temperature', Temp, queue_size = 1)
        self.msgL = Leak()
        self.msgT = Temp()
        self.threshold = threshold
        self.mylock = threading.Lock()
        t = threading.Timer(1, self.report)
        t.start() 
           
    def get_readings(self):
        line = ''
        reading = self.ser.read()
        #now grab one line
        while reading != '\n':
            line += reading
            reading = self.ser.read()
        line = line[:-1] #remove \r from end of line
        return line.split(',')         

    def report(self):
        data = self.get_readings()
        try:
            if data[0] == 'temp': #temperture data
                self.msgT.degrees = float(data[1])
                self.pubTemp.publish(self.msgT)
            elif data[0] == ['A','B']:
                if int(data[1]) < self.threshold:     
                    with self.mylock:
                        rospy.set_param('leak','wet')
                    self.msgL.status = 'wet ' + data[0]
                    self.msgL.val = int(data[1])
                    self.pubLeak.publish(self.msgL)       
                    rospy.loginfo("ardsens.py, Leak in " + data[0] + " Value:" + data[1])
        except Exception, e:
            rospy.loginfo("ardsens.py", e, data)
        t = threading.Timer(1, self.report)
        t.start() 
        
if __name__ == '__main__':
    rospy.init_node('arduino_sensor')
    try:
        port = rospy.get_param('arduino_port')
    except Exception,e:
        rospy.loginfo("ardesns.py, cannot get arduino Port parameter" + e)
    ardSens(port = port, threshold = 300)
    rospy.spin()    
    
        
