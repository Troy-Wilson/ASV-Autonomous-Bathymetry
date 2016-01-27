#!/usr/bin/env python
"""
Created on Mon Feb  3 11:06:33 2014

@author: troy
"""
import threading
import time
import serial
import re
#import rospy

#reading of serial data expects to find comma separated lines terminated by \n
#each line will be split into a tuple based on the comma's, and each tuple
#will be put into the queue

#sending of serial data will work with any string, however of course for this
#module to read and parse the info, above formatting required

class SerialAPI(threading.Thread):
    def __init__(self, rx_data_q, tx_data_q, port, baud, stopbits, parity,
                 timeout,NMEA=False):
        threading.Thread.__init__(self)
        self.port = None
        self.serial_arg = dict(port = port, baudrate=baud, stopbits=stopbits,
                              parity = parity, timeout=timeout)
        #rospy.loginfo(self.serial_arg['port'])
        self.rx_data_q = rx_data_q
        self.tx_data_q = tx_data_q        
        self.NMEA = NMEA
        self.alive = threading.Event()
        self.alive.set()
        
    def run(self):#this overrides the threading run method. called by 
                  #class.start this runs the method in a new thread
        try:
            if self.port:
                self.port.close
            self.port = serial.Serial(**self.serial_arg)
        except serial.SerialException:
            return
        
#        newdata = ""
        line = ""   
                
        while self.alive.isSet(): #self.alive.clear() to break          
            #transmit message if there
            if self.tx_data_q.empty() == False:                 
                tx = self.tx_data_q.get_nowait()
                self.port.write(tx)
                
            #read message if there
 #           if newdata:
 #               line = newdata
 #               newdata = ""
            
            bytesToRead = self.port.inWaiting()
            
            if bytesToRead >0:
                rx_data = self.port.read(bytesToRead)  
                line += rx_data

            if re.search("\r\n",line):
                datatup = line.split("\r\n")
                for i in datatup:                
                    #return datatup[j] as a tuple split by csv
                    
                    if self.NMEA==False:
                        temp = i.split(',')
                    else:
                        #remove first character which is a $ and last 3 characters
                        #that are the checksum
                        temp = i[1:-3].split(',')
   #                 if len(temp) == 3: #assuming each line should have 3 cols
                    self.rx_data_q.put(temp)                   
                    
  #                  elif len(temp) < 3:#if not a full line then buffer
 #                       newdata += i
                    line = ""
            
        if self.port:
            self.port.close()
            
    def join(self,timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
        #i think this causes the main thread to wait until the sub thread is 
        #finished or the timeout is reached
         
if __name__ == '__main__':
    import Queue
    rx_data_q = Queue.Queue()
    tx_data_q = Queue.Queue()        
    serialAPI = SerialAPI(rx_data_q, tx_data_q,
                            '/dev/ttyUSB0',9600,serial.STOPBITS_ONE,
                            serial.PARITY_NONE,1)                                            
    serialAPI.start() 
    time.sleep(0.1) #givethread time to get to while loop
    for i in xrange(100):
        tx_data_q.put(str(i) + "\r\n")
        time.sleep(0.01)
    time.sleep(1)
    serialAPI.join(5)
    serialAPI = None
    