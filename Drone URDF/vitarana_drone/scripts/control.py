#! /usr/bin/env python3
import rospy
import time
from pid import PID_alt
#import read_sensor_data
import message_filters
from vitarana_drone.msg import prop_speed
from rospy.topics import Publisher
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, Float64MultiArray

altitude = 0.30999
kp = 25
ki = 0.05
kd = 20
flag = 0
vel_x = 0 
vel_y = 0 
vel_z = 0
roll = 0
pitch = 0
yaw = 0
message_pub = rospy.Publisher("/edrone/pwm", prop_speed, queue_size=1000)
req_alt = float(input("Enter height drone should hover at : "))
thrust = 0

def setPID(msg):
    global kp,ki,kd
    kp = msg.data[0]
    ki =  msg.data[1]
    kd = msg.data[2]

def calAltitude(msg):
    global altitude
    altitude  = msg.altitude
    rospy.loginfo("\nAltitude = " + str(altitude))
    #return altitude

def calVelocity(msg):
    global vel_x, vel_y, vel_z
    vel_x = msg.vector.x
    vel_y = msg.vector.y
    vel_z = msg.vector.z
    #rospy.loginfo("\nVx = {0}\nVy = {1}\nVz = {2}\n".format(vel_x,vel_y,vel_z))
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
    #rospy.loginfo("\nRoll = {0}\nPitch = {1}\nYaw = {2}\n".format(roll,pitch,yaw))    
    #rospy.loginfo("\nWx = {0}\nWy = {1}\nWz = {2}\n".format(angVel_x,angVel_y,angVel_z))
    
    #return (roll, pitch, yaw)



def PID_alts(gps, vel, imu):
    
    global altitude #!!
    global req_alt
    global flag
    global thrust
    global speed
    global rate
    global prev_time,prev_alt_err,i_term,d_term,p_term
    global roll, pitch, yaw
    global flag
    altitude  = gps.altitude
    print("\nAltitude = " + str(altitude))
    current_alt_err = req_alt - altitude
    print("Required alt = ",req_alt)
    # rospy.init_node("pid_alt_node",anonymous=False)
    rospy.Subscriber("alt_pid", Float64MultiArray, setPID) #!!
    #calVelocity(vel)
    #calImu(imu)

    
    # print("roll - control",roll)
    # print("pitch - control",pitch)
    # print("yaw - control",yaw)
    
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

    print("Altitude Correction = ",output_alt)
    thrust = hover_speed + output_alt*10

    #we need to limit this thrust
    if(thrust > 1000): 
        thrust = 1000
    elif(thrust < 0):
        thrust = 0
    
    print("Thrust = ",thrust)

    speed = prop_speed()
    speed.prop1 = thrust
    speed.prop2 = thrust
    speed.prop3 = thrust
    speed.prop4 = thrust
    rospy.loginfo(speed) #!!!
    message_pub.publish(speed)

    # while not rospy.is_shutdown():
    #     message_pub.publish(speed)


def alt_control(gps, vel, imu):
    global altitude #!!
    global req_alt
    global flag
    global kp,ki,kd
    # global thrust
    # global speed
    global roll, pitch, yaw

    calVelocity(vel)
    calImu(imu)
    calAltitude(gps)
    rospy.Subscriber("alt_pid", Float64MultiArray, setPID) #!!
    k_alt = (kp,ki,kd)
    print("\nAltitude = " + str(altitude))
    current_alt_err = req_alt - altitude
    print("Required alt = ",req_alt)
    print("Roll =", roll)
    print("Pitch =", pitch)
    print("Yaw =", yaw)
    # rospy.init_node("pid_alt_node",anonymous=False)
    #rospy.Subscriber("alt_pid", Float64MultiArray, setPID) #!!
    
    #get information of the velocity and r p y of the drone
    
    #the goal is to get a function that stabilises the r p y of the drone while maintaining altitude
    speed = PID_alt(roll, pitch, yaw, req_alt, altitude, k_alt, flag)
    flag += 1
    message_pub.publish(speed)





def control():
    global altitude, thrust, speed
    rospy.init_node("altitude", anonymous = False)
    gps_sub = message_filters.Subscriber("/edrone/gps", NavSatFix)
    vel_sub = message_filters.Subscriber("/edrone/gps_velocity", Vector3Stamped)
    imu_sub = message_filters.Subscriber("/edrone/imu/data", Imu)
    ts = message_filters.TimeSynchronizer([gps_sub, vel_sub, imu_sub], 2)
    #one of these publishers is slower than the others
    #which is why the messages are loading relatively slowly
    ts.registerCallback(alt_control)

    rospy.spin()
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

    






