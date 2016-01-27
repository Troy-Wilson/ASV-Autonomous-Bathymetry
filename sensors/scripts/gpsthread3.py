#import roslib; roslib.load_manifest('sensors')
import serial, time
import threading
import re
import rospy

class Gpsthread(threading.Thread):
    def __init__(self,port1,port2,port3, pause = 0.05, baud = 19200):
        threading.Thread.__init__(self)
        self.pause = pause
        self.usb = [0]*3             
        self.usb[0] = serial.Serial(port=port1, timeout = 1, baudrate = baud)
        self.usb[1] = serial.Serial(port=port2, timeout = 1, baudrate = baud)
        self.usb[2] = serial.Serial(port=port3, timeout = 1, baudrate = baud)
        self.kill = False
        self.northing = 0
        self.northing_sd = 0 
        self.easting = 0
        self.easting_sd = 0
        self.height = 0
        self.height_sd = 0
        self.lat = 0
        self.long = 0
        self.lat_sd = 0
        self.long_sd = 0
        self.zone = 0
        self.band = ''
        self.satellites = 0
        self.sol_status = -2
        #self.text1 = ''
        self.buff = ['']*3
        self.count = 0
        try:
            self.gpsZero = rospy.get_param("GPS_zero")
        except:
            self.gpsZero = [0.0,0.0]
        self.eastingZero = self.gpsZero[0]
        self.northingZero = self.gpsZero[1]

        #time.sleep(1)
    
    def tare(self):
        self.start_northing = self.northing
        self.start_easting = self.easting
                    
#    def get_readings(self, portno, timeout = 0.5):
#        #slow as only grabs 1 character at a time, prob ok for this data however
#        start = time.time()
#        line = ''
#        reading = self.usb[portno].read()
#        
#        #grab one line
#        while reading != '\n':
#            if (time.time()-start) >timeout:
#                return None
#            line += reading
#            reading = self.usb[portno].read()
#        line = line[:-2] #remove '\r from end of line
#        lines = re.split(',|;', line)    
#        return lines
        
    def buffread(self, portno, timeout = 1.0):
        self.buff[portno] += self.usb[portno].read(size=100) #read 100chrs at a time
        lines = self.buff[portno].split('#')
        lcnt = len(lines)

        if lcnt>1:          
            br = lines[0]
            br = br[:-2] #remove '*XX\r\n from end of line           
            self.buff[portno] = lines[lcnt-1]
            return re.split(',|;', br)  
        else:
            return None
            
        
    def alloc_readings(self,readings):
        try:
            length = len(readings)
            if readings[0] == 'BESTUTMA': 
                if length >= 26:
                    if readings[10] == 'SOL_COMPUTED':
                        if self.sol_status == -1:
                            #first initialisation, wait for GPS to get accurate fix, 2 min
                            time.sleep(120)
                        self.sol_status = 0
    
                    elif readings[10] == 'INSUFFICIENT_OBS':
                        self.sol_status = -1
    
                    self.zone = int(readings[12])
                    self.band = readings[13]
                    self.northing = float(readings[14])-self.northingZero
                    self.easting = float(readings[15])-self.eastingZero
                    self.height = float(readings[16])
                    self.northing_sd = float(readings[19])
                    self.easting_sd = float(readings[20])
                    self.height_sd = float(readings[21])
                    self.satellites = int(readings[25])
                    self.count = float(readings[6])
                    #print "n ", self.northing, " e ", self.easting, " secs " , self.count, time.time()
                
            elif readings[0] == '#BESTPOSA':
                if length >= 20:
                    self.lat = float(readings[12])
                    self.long = float(readings[13])
                    #self.height = float(readings[14])
                    self.lat_sd = float(readings[17]) 
                    self.long_sd = float(readings[18]) 
                    #self.height_sd = float(readings[19]) 
            
        except Exception as e:   
            #print readings
            rospy.logwarn(readings)
            rospy.logwarn(e)
            
            
            
    def write(self,code, portno):
        self.usb[portno].write(code)

    def close(self):
        self.usb[0].close
        self.usb[1].close
        #self.usb[0].close
        
    def run(self):   
        self.write('LOG BESTUTMA ONTIME 1\r',0)
        self.write('LOG BESTPOSA ONTIME 1\r',1)
        while self.kill == False:
            #temp0 = self.get_readings(0) 
            #temp1 = self.get_readings(1) 
            #temp2 = self.get_readings(2) 
            temp0 = self.buffread(0)
            temp1 = self.buffread(1)
        
            #print temp 
            #print 'temp0' , temp0
            #print 'temp1' , temp1
            if temp0 != None:            
                self.alloc_readings(temp0)
            if temp1 != None:
                self.alloc_readings(temp1)
#            if temp2 != None:
#                    self.alloc_readings(temp2)
            time.sleep(self.pause) #gps unit only runs at 1Hz
        self.close()        

if __name__ == '__main__':

    gps = Gpsthread(port1='/dev/gps0',port2='/dev/gps1',port3='/dev/gps2' )
    gps.start()
    time.sleep(10)
    gps.kill = True
    gps.close()
#    print "easting, northing, lat, long, sol_status, number_satellites"
#    for i in range(30):
#        print gps.easting, gps.northing, gps.lat, gps.long, gps.sol_status, gps.satellites
#        time.sleep(1)
#    gps.kill = True
#    gps.close()
#    print 'done'

