#! /usr/bin/env python3

import rospy
import time
from vitarana_drone.msg import prop_speed
from rospy.topics import Publisher
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, Float64MultiArray

altitude = 0.30999
kp = 2
ki = 0.002
kd = 200 
flag = 0
message_pub = rospy.Publisher("/edrone/pwm", prop_speed, queue_size=1000)
req_alt = float(input("Enter height drone should hover at : "))
thrust = 0

def setPID(msg):
    global kp,ki,kd
    kp = msg.data[0]
    ki =  msg.data[1]
    kd = msg.data[2]

def PID_alt(msg):
    global altitude #!!
    global req_alt
    global flag
    global thrust
    global speed
    global rate
    global prev_time,prev_alt_err,i_term,d_term,p_term
    altitude  = msg.altitude
    print("\nAltitude = " + str(altitude))
    current_alt_err = req_alt - altitude
    print("Required alt = ",req_alt)
    # rospy.init_node("pid_alt_node",anonymous=False)
    rospy.Subscriber("alt_pid", Float64MultiArray, setPID) #!!

    print("kp = ",kp)
    print("ki = ",ki)
    print("kd = ",kd)
    hover_speed = 508.75
    sample_time = 0
    current_time = time.time()
    # print("Current time = ",current_time)
    if flag == 0:
        prev_time = 0
        prev_alt_err = 0
        i_term = 0
        d_term = 0
        

    dTime = current_time - prev_time
    dErr_alt = current_alt_err -prev_alt_err

    if (dTime >= sample_time):
        if flag==0:
            p_term = 0
            i_term = 0
            d_term = 0
            flag+=1
        else:
            p_term = current_alt_err
            i_term += current_alt_err * dTime
            d_term =  dErr_alt/dTime

    prev_time = current_time
    prev_alt_err = current_alt_err

    output_alt = kp*p_term + ki*i_term + kd*d_term

    thrust = hover_speed + output_alt*1.5
    print("Thrust = ",thrust)

    speed = prop_speed()
    speed.prop1 = thrust
    speed.prop2 = thrust
    speed.prop3 = thrust
    speed.prop4 = thrust
    rospy.loginfo(speed) #!!!


    while not rospy.is_shutdown():
        message_pub.publish(speed)


def control():
    global altitude, thrust, speed
    rospy.init_node("altitude", anonymous = False)
    rospy.Subscriber("/edrone/gps", NavSatFix, PID_alt)
    # rospy.init_node("motor_speed_pub",anonymous=False)
    # speed = prop_speed()
    # speed.prop1 = thrust
    # speed.prop2 = thrust
    # speed.prop3 = thrust
    # speed.prop4 = thrust
    # rospy.loginfo(speed) #!!!
    # while not rospy.is_shutdown():
    #     message_pub.publish(speed)
       


if __name__=='__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
        


# Alt input from user = req_alt = drone hover at that alt
# Sub_gps = current_alt
# Sub_Imu = curent r p y
# rqt_console (sliders for kp,ki,kd) ->rostopic(pid_topic)->Sub(take in the kp,ki,kd(enter deafult vals))
# x -> forward R
# y->left G
# z->forward B
# prop1 = front left
# prop2 = front right
# prop3 = back right
# prop4 = back left

# alt_err = req_alt - current_alt 

# calAltPID(msg):
#     global kp,ki,kd
#     (kp,ki,kd) = msg_list

# PID_alt(alt_err)
#     hover_speed = 508.75  
#     Sub("pid_alt",calAltPID)
#     if 0th iteration:
#         current_alt_err = 0
#         prev_time = 0
#         prev_alt_err = 0
#         i_term = 0
    
#     derr = current_alt_err - prev_alt_err
#     dtime = current_time - prev_time
#     current_alt_err = alt_err

#     p_term = kp*alt_err
#     d_term = kd*derr/dtime
#     i_term += ki*dtime    
    
#     output_err = p_term + i_term + d_term

#     thrust = hover_speed + output_err*1.5
#     return thrust




# 	#fr in my code is fl in gazebo's world
# 	motor_fr = thrust + output_yaw + output_pitch + output_roll  
# 	#fl in my code is bl in gazebo's world
# 	motor_fl = thrust - output_yaw + output_pitch - output_roll  
#     #br in my code is fr in gazebo's world
# 	motor_br = thrust - output_yaw - output_pitch + output_roll  
# 	#bl in my code is br in gazebo's world
# 	motor_bl = thrust + output_yaw - output_pitch - output_roll 

    






