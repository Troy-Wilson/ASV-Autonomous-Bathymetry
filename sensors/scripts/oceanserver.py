#!/usr/bin/env python
"""
Created on Wed Feb 12 09:33:52 2014

@author: troy
"""
#import roslib; roslib.load_manifest('sensors')
import rospy
from sensors.msg import Battery
import SerialAPI_bat as SerialAPI
import threading, Queue, serial

class BattPub(threading.Thread):
    def __init__(self,port='/dev/ttyUSB2', baud=19200, timeout=1):
        self.rx_data_q = Queue.Queue() #note this creates a FIFO queue
        self.tx_data_q = Queue.Queue()        
        try:
            self.port = rospy.get_param('batt_port')
        except Exception,e:
            rospy.loginfo("oceanserver.py, cannot get battery Port parameter errcode %s",  e)
        self.serialAPI = SerialAPI.SerialAPI(self.rx_data_q, self.tx_data_q,
                            self.port,baud,serial.STOPBITS_ONE,serial.PARITY_NONE
                            ,timeout,True)                                            
        self.pubBatt = rospy.Publisher('battData', Battery, queue_size=1)
        self.msg = Battery()
        self.serialAPI.start() 
        self.tx_data_q.put(chr(32))
        self.tx_data_q.put('x')        
        t = threading.Timer(1, self.battReport)
        t.start() 


    def battReport(self):
        data = None        
        while self.rx_data_q.empty() == False:            
            data = self.rx_data_q.get()                 
            equipment = data.pop(0) #note this takes the first element out of the list
            elements = len(data)            
            for i in range((elements)/2):
                try:
                    code = int(data[i*2],16) #converst from hex to int using base16
                    val = int(data[i*2+1],16)
                    if equipment[0]=='B': #battery
                        if equipment[2]=='1': #battery 1
                            if code == 13:
                                self.msg.B1RelativeStateOfCharge = val #percent
                            elif code == 9:
                                self.msg.B1Voltage = val/1000.0 #V
                            elif code == 10:
                                self.msg.B1Current = val/1000.0 #A
                            elif code ==15:
                                self.msg.B1RemainingCapacity=val/1000.0 #Ah
                            elif code == 8:
                                self.msg.B1Temp = val/10.0-273.15#convert to celsius
                            elif code == 18:
                                self.msg.B1AvgTimeToEmpty = val #minutes
                            elif code == 22:
                                self.msg.B1Status = self.batStatusString(val)
                            else: pass                            
                        elif equipment[2]=='2':#battery 2
                            if code == 13:
                                self.msg.B2RelativeStateOfCharge = val #percent
                            elif code == 9:
                                self.msg.B2Voltage = val/1000.0
                            elif code == 10:
                                self.msg.B2Current = val/1000.0
                            elif code ==15:
                                self.msg.B2RemainingCapacity=val/1000.0
                            elif code == 8:
                                self.msg.B2Temp = val/10.0-273.15#convert to celsius
                            elif code == 18:
                                self.msg.B2AvgTimeToEmpty = val
                            elif code == 22:
                                self.msg.B2Status = self.batStatusString(val)
                            else: pass
                    elif equipment[0]=='S': #system
                        if code== 1:
                            self.msg.SMinToEmpty = val #minutes
                        elif code ==4:
                            self.msg.SPerCharge = val #percent                        
                    elif equipment[0]=='C': #controller
                        pass
                except Exception, e:
                    rospy.loginfo("oceanserver.py error code %s", e)
                    rospy.loginfo(code)
                    rospy.loginfo(val)                        
            self.pubBatt.publish(self.msg)
        t = threading.Timer(1, self.battReport)
        t.start() 
        
    def batStatusString(self,no):
        if no ==4:
            ret = "Fully Discharged"
        elif no ==5:
            ret = "Fully Charged"
        elif no ==6:
            ret = "Discharging"
        elif no ==7:
            ret = "Initialised"
        elif no ==8:
            ret = "Remaining Time Alarm"
        elif no ==9:
            ret = "Remaining Capacity Alarm"
        elif no ==11:
            ret = "Terminate Discharge Alarm"
        elif no ==12:
            ret = "Over Temp Alarm"
        elif no ==14:
            ret = "Terminate Charge Alarm"
        elif no ==15:
            ret = "Over Charged Alarm"
        else:
            ret = str(no)
        return ret
        
if __name__ == '__main__':
    rospy.init_node('BatteryInfo')
    BattPub()
    rospy.spin()            

    
    
    
    