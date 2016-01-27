# -*- coding: utf-8 -*-

import motorcont
import time

m1 = motorcont.MotorCont(port='/dev/pololu_starboard')

for i in range(0,200,1):
    if i < 100:        
        m1.throttle(i)
    else:
        m1.throttle(200-i)           
    print i
    time.sleep(0.1)
 
for i in range(0,200,1):
    if i < 100:        
        m1.throttle(-i)
    else:
        m1.throttle(-200+i)           
    print i
    time.sleep(0.1) 
    
m1.close()

