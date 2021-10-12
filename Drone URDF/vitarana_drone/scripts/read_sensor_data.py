#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64

roll = 0
pitch = 0
yaw = 0

def calAltitude(msg):
    altitude  = msg.altitude
    rospy.loginfo("\nAltitude = " + str(altitude))
    #return altitude

def calVelocity(msg):
    vel_x = msg.vector.x
    vel_y = msg.vector.y
    vel_z = msg.vector.z
    rospy.loginfo("\nVx = {0}\nVy = {1}\nVz = {2}\n".format(vel_x,vel_y,vel_z))
    #return (vel_x, vel_y, vel_z)

def calImu(msg):
    orinetation_list = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    global roll, pitch, yaw 
    (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
    roll = roll * (180/3.14159265)
    pitch = pitch * (180/3.14159265)
    yaw = yaw * (180/3.14159265)
    angVel_x = msg.angular_velocity.x
    angVel_y = msg.angular_velocity.y
    angVel_z = msg.angular_velocity.z
    rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))    
    rospy.loginfo("\nWx = {0}\nWy = {1}\nWz = {2}\n".format(angVel_x,angVel_y,angVel_z))
    
    #return (roll, pitch, yaw)

def getImu():
    global roll, pitch, yaw
    return (roll, pitch, yaw)



def listener():
    rospy.init_node("listener", anonymous = False)
    rospy.Subscriber("/edrone/gps", NavSatFix, calAltitude)
    rospy.Subscriber("/edrone/gps_velocity", Vector3Stamped, calVelocity)
    rospy.Subscriber("/edrone/imu/data", Imu, calImu)
    rospy.spin()

    

if __name__=='__main__':
    listener()

    
    

