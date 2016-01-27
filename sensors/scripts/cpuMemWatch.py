
#!/usr/bin/env python
from __future__ import division

__author__ = 'troy'

import psutil
from sensors.msg import cpumem
import rospy
import os.path

def cpumemWatch():
    rospy.init_node('CPUMem')
    cpumemMsg = cpumem()
    pub = rospy.Publisher('cpuMem', cpumem, queue_size=1)
    userHome = os.path.expanduser('~')
    memOpt = open((userHome + '/tmp/memOpt.txt'),'w')


    while True:
        cpumemMsg.cpuPercentPerCore = psutil.cpu_percent(1.0, True)
        cpumemMsg.memPercent = psutil.virtual_memory().percent
        pub.publish(cpumemMsg)
        outStr = ''
        for corePer in cpumemMsg.cpuPercentPerCore:
            outStr += str(corePer) + ','
        outStr += str(cpumemMsg.memPercent) + '\n'
        memOpt.write(outStr)

if __name__ == '__main__':
    try:
        cpumemWatch()
    except rospy.ROSInterruptException:
        pass



