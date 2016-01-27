__author__ = 'troy'

import imuthread3
import os.path
import time
try:
    userHome = os.path.expanduser('~')
    dir = userHome + '/tmp/'
    kfdata = open((dir + 'KFout.txt'),'w')

    vn = imuthread3.Imuthread(port='/dev/imu', pause=0.001)
    vn.start()
    vn.set_data_freq50()
    vn.set_qmr_mode()
    #vn.set_data_freq10() #to see if this fixes my gps drop out problem
    time.sleep(3)
    print "started recording"

    while True:
        if len(vn.lastreadings)>0:
            if vn.lastreadings[0] =='VNQMR':
                timestamp = time.time()
                qx = float(vn.lastreadings[1])
                qy = float(vn.lastreadings[2])
                qz = float(vn.lastreadings[3])
                qw = float(vn.lastreadings[4])
                xdd = float(vn.lastreadings[11])
                ydd = float(vn.lastreadings[12])
                zdd = float(vn.lastreadings[13])
                kfstr = '{:.6f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f}'.format(timestamp,qx,qy,qz,qw,xdd,ydd,zdd) + '\n'
                kfdata.write(kfstr)
                while time.time() - timestamp < 1.0/50:
                    time.sleep(0.001)

except KeyboardInterrupt:
    print "keyboard interrupt detected, process killed"
    vn.kill
    kfdata.close()


