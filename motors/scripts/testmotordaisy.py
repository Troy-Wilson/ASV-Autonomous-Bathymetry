__author__ = 'troy'

import motorcontdaisy
import time


portID = '\x01'
starID = '\x02'
mc = motorcontdaisy.MotorContDaisy('/dev/pololu_ttl',)
rest = 0.05

print "running port(left) motor"

for i in range(0,200,1):
    if i < 100:
        mc.throttlePort(i)
    else:
        mc.throttlePort(200-i)
    print i
    time.sleep(rest)

for i in range(0,200,1):
    if i < 100:
        mc.throttlePort(-i)
    else:
        mc.throttlePort(-200+i)
    print i
    time.sleep(rest)

print "running starboard(right) motor"

for i in range(0,200,1):
    if i < 100:
        mc.throttleStar(i)
    else:
        mc.throttleStar(200-i)
    print i
    time.sleep(rest)

for i in range(0,200,1):
    if i < 100:
        mc.throttleStar(-i)
    else:
        mc.throttleStar(-200+i)
    print i
    time.sleep(rest)

mc.close()
