#!/usr/bin/env python
#import roslib; roslib.load_manifest('sensors')
import rospy
import tf
import imuthread3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def Vn100Pub():
    pub = rospy.Publisher('IMUData', Imu, queue_size=1)
    pub2 = rospy.Publisher('IMUMag', MagneticField, queue_size=1)
    # Initialize the node and name it.
    rospy.init_node('IMUpub')
    vn = imuthread3.Imuthread(port=rospy.get_param("imu_port"), pause=0.0)

    vn.start()
    vn.set_data_freq50()
    vn.set_qmr_mode()
    #vn.set_data_freq10() #to see if this fixes my gps drop out problem
    rospy.sleep(3)
    msg = Imu()
    msg2 = MagneticField()
    count = 0
    while not rospy.is_shutdown():
        if len(vn.lastreadings)>0:
            if vn.lastreadings[0] =='VNQMR':
                msg.header.seq = count
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'imu'
                msg.orientation.x = float(vn.lastreadings[1])
                msg.orientation.y = float(vn.lastreadings[2]) 
                msg.orientation.z = float(vn.lastreadings[3])
                msg.orientation.w = float(vn.lastreadings[4])
                msg.orientation_covariance = [0,0,0,0,0,0,0,0,0]
                msg.angular_velocity.x = float(vn.lastreadings[8])
                msg.angular_velocity.y = float(vn.lastreadings[9])
                msg.angular_velocity.z = float(vn.lastreadings[10])
                msg.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
                msg.linear_acceleration.x = float(vn.lastreadings[11])
                msg.linear_acceleration.y = float(vn.lastreadings[12])
                msg.linear_acceleration.z = float(vn.lastreadings[13])
                msg.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]
                msg2.header.seq = count
                msg2.header.stamp = rospy.Time.now()
                msg2.header.frame_id = 'imu'
                msg2.magnetic_field.x = float(vn.lastreadings[5])
                msg2.magnetic_field.x = float(vn.lastreadings[6])
                msg2.magnetic_field.x = float(vn.lastreadings[7])
                msg2.magnetic_field_covariance =  [0,0,0,0,0,0,0,0,0]

            #rospy.loginfo("vn100_pub " + str(msg.header.stamp) + " " + str(msg.orientation.x) + " "+ str(msg.orientation.y) + " "+ str(msg.orientation.z) + " "+ str(msg.orientation.w) + " ")
                current_pose_euler = tf.transformations.euler_from_quaternion([
                                    msg.orientation.x,
                                    msg.orientation.y,
                                    msg.orientation.z,
                                    msg.orientation.w
                                    ])                        
                pub.publish(msg)
                pub2.publish(msg2)
                count += 1
        #rospy.sleep(1.0/100.0)            
    vn.kill = True
        
if __name__ == '__main__':
    try:
        Vn100Pub()
    except rospy.ROSInterruptException: 
        pass        
