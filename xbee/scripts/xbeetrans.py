#!/usr/bin/env python
"""
Created on Fri Jan 17 10:23:00 2014

@author: troy
"""
#import roslib; roslib.load_manifest('xbee')
import serial
import Queue
import SerialAPI
import time

class XbeeTrans:
    def __init__(self,port='/dev/ttyUSB0'):
        self.rx_data_q = Queue.Queue() #note this creates a FIFO queue
        self.tx_data_q = Queue.Queue()        
        self.serialAPI = SerialAPI.SerialAPI(self.rx_data_q, self.tx_data_q,
                            port,38400,serial.STOPBITS_ONE,serial.PARITY_NONE,1)                                            
        self.serialAPI.start() 
        self.today = (round(time.time()/(60*60*24)-.5,0))*(60*60*24) #note using round(f,-0.5) = floor(f)
        
        # Once this is running, anything put into tx_data_q will be transmitted
        # across the serial port, and anything that is received over the serial
        # port will appear in rx_data_q               
        
    def close(self):
        self.serialAPI.join(0.01) 
        self.serialAPI = None
            
    def transmit(self, blk):        
        #will send 3 feilds, name, data, time. CSV delimited, newline terminated
        adjtime = str(round(time.time()-self.today,2))
        if blk[1] != None:        
            self.tx_data_q.put(blk[0]+','
                               +str(blk[1])+','
                               +adjtime+'\r\n')
                               #+time.strftime("%T")+'\r\n')
       
    def read_data(self):
        data = None        
        if self.rx_data_q.empty() == False:
            #data += self.rx_data_q.get() + ','
            data = self.rx_data_q.get()
            #print("from xtrans.py read_data() = " + str(data))
        return data
        
if __name__ == '__main__':
    x = XbeeTrans()
    #print("xbee initialised")
    for i in range(3):    
        x.transmit(("temp", i*10))
        time.sleep(0.1)
    #x.transmit_blk("hello world ")
    time.sleep(1)
    x.close()
    print("done")

