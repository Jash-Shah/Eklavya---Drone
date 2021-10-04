#! /usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


def fly():
    rospy.init_node('my_node_move')

    empty = Empty()
    var_twist = Twist()
    pub_position = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    var_twist.linear.x = 3
    var_twist.linear.y = 3
    var_twist.linear.z = 0
    takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    
    while not rospy.is_shutdown():
        #takeoff.publish(empty)
        #pass
        pub_position.publish(var_twist)
        

if __name__ == '__main__':
    try:
        fly()
    except rospy.ROSInterruptException:
        pass


#pub_position.publish(var_twist)


