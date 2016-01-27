import time, threading, serial
import numpy as np

# main funtions to use are
# Imuthread.start()
# Imuthread.lastreadings
# Imuthread.kill = True
# Imuthread.tare()

class Imuthread(threading.Thread):
    def __init__(self,port='/dev/ttyUSB0', baud=115200, timeout=1, pause = 1.0/200):
        threading.Thread.__init__(self)
        self.ser = serial.Serial(
                                 port,
                                 baudrate = baud,
                                 timeout = timeout,
                                 parity = serial.PARITY_NONE, 
                                 rtscts = False, 
                                 xonxoff = False,
                                 bytesize = 8
                                 )
        self.ser.flush()
        self.kill = False
        self.pause = pause
        self.lastreadings = []
        self.buff = ''
           
#    def get_readings(self):
#        line = ''
#        reading = self.ser.read()
        #loop through to first character on line        
#        while reading != '$':
#            reading = self.ser.read()
            #print reading
        #now grab one line
#        while reading != '\n':
#            line += reading
#            reading = self.ser.read()
#        line = line[:-4] #remove '*x\r from end of line 
#        #return numpy.array(line.split(','))
#        return line.split(',')         
    
    def buffread(self):
        self.buff = self.buff + self.ser.read(size=100) #read 100chrs at a time
        lines = self.buff.split('$')
        lcnt = len(lines)
        if lcnt>2:
            br = lines[lcnt-2]
            br = br[:-5] #remove '*XX\r\n from end of line           
            self.buff = lines[lcnt-1]
            return br.split(',')
        else:
            return None
    
    def tare(self):
        self.ser.write('$VNTAR*5F\n')
        time.sleep(0.5)
        self.ser.flushOutput()

    def run(self):
        time.sleep(2)
        while self.kill==False:                   
            br = self.buffread()
            if br != None:
                areNum = True
                vals = np.empty([len(br)],dtype=object)
                vals[0] = br[0] #this will be a text string
                for i in range(1,len(br)):
                    # if any of the readings cannot be converted to num, then discard the lot
                    try:
                        vals[i] = float(br[i])
                    except ValueError:
                        areNum = False
                        break
                if areNum == True:
                    self.lastreadings = vals
            time.sleep(self.pause)
        self.ser.close()
        
    def get_labels(self):
        tempar = self.get_readings()
        if tempar[0] == '$VNYMR':
            labels = ['Type','Yaw','Pitch','Roll','MagX','MagY','MagZ','AccelX',
                   'AccelY','AccelZ','GyroX','GyroY','GyroZ']
        else: 
            labels = [] 
        return labels
        
#    def get_yaw(self):
#        tempar = self.get_readings()
#        for j in range(len(tempar)):
#            print tempar[j], 
#        print '/n'
#        if tempar[0] == ('$VNYMR' or '$VNYPR'):
#            #return numpy.asscalar(numpy.float64(tempar[1]))
#            return float(tempar[1])
#        else:
#            return None
    
#    def set_ypr_mode(self):
#        self.ser.write(('$VNWRG,6,1*5D\n'))

    def set_ymr_mode(self):
        #see page 66 of manual (table 26)        
        self.ser.write('$VNWRG,6,14*69\n')

    def set_qmr_mode(self):
        #see page 66 of manual (table 26)
        self.ser.write('$VNWRG,6,8*54\n')

    def set_qtn_mode(self):
        #return quaternion only
        self.ser.write('$VNWRG,6,2*5E\n')

    def set_qta_mode(self):
        #return quaternion and acceleration
        self.ser.write('$VNWRG,6,4*58\n')

    def set_qar_mode(self):
        #return quaternion, acceleration and angular
        self.ser.write('$VNWRG,6,7*5B\n')

    def set_acc_mode(self):
        #return acceleration data
        self.ser.write('$VNWRG,6,11*6C\n')

    def set_gyr_mode(self):
        #return angular rate data
        self.ser.write('$VNWRG,6,12*6F\n')

    def write_settings_to_flash(self):
        self.ser.write("$VNWNV*57\n")    
        #not tested        
        
    def reset(self):
        self.ser.write('$VNRST*4D\n')
        
    def restore_factory_settings(self):
        self.ser.write('$VNRFS*5F\n')
        
    def set_data_freq10(self):
        #self.ser.write('$VNWRG,7,10*XX\n')
        self.ser.write('$VNWRG,7,10*6C\n')

    def set_data_freq1(self):
        self.ser.write('$VNWRG,7,1*5C\n')
        
    def set_data_freq50(self):
        self.ser.write('$VNWRG,7,50*68\n')
        
    def set_data_freq5(self):
        self.ser.write('$VNWRG,7,5*58\n')

    def set_vpe(self,HeadingMode = 0):
        self.ser.flush()
        if HeadingMode == 0:
            self.ser.write('$VNWRG,35,1,0,1,1*71\n')
        elif HeadingMode == 1:
            self.ser.write('$VNWRG,35,1,1,1,1*70\n')
        else:
            self.ser.write('$VNWRG,35,1,2,1,1*73\n')
        print self.buffread()
        #VPE = 1 Enable
        #Headingmode = 0 Absolute Heading, 1 relative, 2 indoor
        #filtering Mode = 1 Mode 1
        #tuning mode = 1 mode 1

    def read_vpe(self):
        self.ser.flush()
        self.ser.write('$VNRRG,35*75\n')
        print self.buffread()

if __name__ == '__main__':
    vn = Imuthread(port="/dev/imu", pause=0.0, timeout = 0.1)
    vn.set_qmr_mode()
    time.sleep(1)
    vn.set_data_freq10()
    time.sleep(1)
    vn.start()
    for i in range(100):
        print vn.lastreadings
        time.sleep(0.1)
    vn.kill = True

    