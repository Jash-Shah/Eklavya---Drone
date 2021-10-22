#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from vitarana_drone.msg import prop_speed

def takeoff():
    pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=10)
    rospy.init_node('takeoff_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    speed = prop_speed()
    speed.prop1 = 510
    speed.prop2 = 510
    speed.prop3 = 510
    speed.prop4 = 510
    i = 0
    while not rospy.is_shutdown():
        if i == 10:
            speed.prop1 = 508.5
            speed.prop2 = 508.5
            speed.prop3 = 508.5
            speed.prop4 = 508.5
        rospy.loginfo(speed)
        pub.publish(speed)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass