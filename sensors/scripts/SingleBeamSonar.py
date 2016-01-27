#!/usr/bin/env python
__author__ = 'Troy Wilson'

import serial
import time
import threading
import rospy
from sensors.msg import SBS
import time

# parsed data is
# 0 = SDDPT : Depth assuming a given offset of transduecer (zero in this case) in meters
# 1 = SDDBT : Depth Below Transducer depth feet, depth meters, depth fathoms
# 2 = YXMTW : Temperature, celsius

#publishes -9999 if there is an empty value. This need to be caught on the other side

class SingleBeamSonar(threading.Thread):
    def __init__(self, port='/dev/s_sonar'):

        threading.Thread.__init__(self)
        rospy.init_node('SingleBeamSonar')
        self.ser = serial.Serial(
                                 port,
                                 baudrate = 4800,
                                 timeout = 0.01,
                                 parity = serial.PARITY_NONE,
                                 rtscts = False,
                                 xonxoff = False,
                                 bytesize = 8
                                 )
        self.ser.flush()

        self.kill = False
        self.pub = rospy.Publisher('SBSData', SBS, queue_size=1)
        self.msg = SBS()
        # Initialize the node and name it.

        self.start()

    # def buffread(self):
    #     #this worked on the moxa rs422 to usb convertor but the FTDI one is returning some bad characters as well
    #     rd = self.ser.read(size=150)
    #     if len(rd) > 0:
    #         lines = rd.split('$')
    #         lcnt = len(lines)
    #         if lcnt>=3:
    #             temp = lines[2].split(',')
    #             temp = temp[3]
    #             if self.isNumber(temp):
    #                 self.msg.waterTemp = float(temp)
    #             else:
    #                 self.msg.waterTemp = -9999
    #         if lcnt>=4:
    #             temp = lines[3].split(',')
    #             temp = temp[1]
    #             if self.isNumber(temp):
    #                 self.msg.range = float(temp)
    #             else:
    #                 self.msg.range = ""
    #         self.pub.publish(self.msg)

    def buffreadftdi(self):
        #hacking around extra bits being returned. probably need to use a CRO to align the timing of the chip
        rd = self.ser.read(size=150)
        lines = rd.split('$')
        lcnt = len(lines)
        #print lcnt, lines
        newdata = False
        if lcnt >1: #to ignore bad data returns
            lines = lines[1:]
            for line in lines:
                splitline = line.split(',')
                if len(splitline) >1:
                    #print splitline

                    if splitline[0] == 'SDDPT':
                        pass
                    elif splitline[0] == 'SDDBT':
                        if len(splitline)>3:
                            if self.isNumber(splitline[3]):
                                data = splitline[3]
                            else:
                                data = -9999.9
                        else:
                            data = -9999.9
                        newdata = True
                        self.msg.range = float(data)
                    elif splitline[0] == 'YXMTW':
                        if self.isNumber(splitline[1]):
                            data =  splitline[1]
                        else:
                            data = -9999.9
                        newdata = True
                        self.msg.waterTemp = float(data)
        if newdata == True:
            self.pub.publish(self.msg)
        time.sleep(0.1)

    def run(self):
        while self.kill==False:
            self.buffreadftdi()
        self.ser.close()

    def isNumber(self,s):
        try:
            float(s)
            return True
        except ValueError:
            return False

if __name__ == '__main__':
    try:

        SingleBeamSonar()
    except rospy.ROSInterruptException:
        pass
