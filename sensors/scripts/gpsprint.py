import gpsthread3
import time
#import sys

#if len(sys.argv) == 2:
#    duration = int(sys.argv[1])
#else:
#    duration = 10

try:    
    gps = gpsthread3.Gpsthread(port1='/dev/gps0', port2='/dev/gps1',port3='/dev/gps2')
    gps.start()
    time.sleep(2)
    count = 0
#    print "run for " , duration , " seconds"
    print "easting, northing, lat, long, sol_status, number_satellites"
#    for i in range(duration):
    while True:
        if gps.count != count:
            print gps.easting, gps.northing, gps.lat, gps.long, gps.sol_status, gps.satellites, gps.count
        count = gps.count
        time.sleep(0.1)
    gps.kill = True
    gps.close
    print 'done'
except KeyboardInterrupt:
    print "KeyboardInterrupt detected"
    gps.kill = True
    gps.close
    print 'done'
