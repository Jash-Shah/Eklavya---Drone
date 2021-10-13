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


def PID_alt(roll, pitch, yaw, req_alt, altitude, k_alt, k_roll, k_pitch, k_yaw, current, target, velocity, flag): 
    #global variables are declared to avoid their values resetting to 0
    global prev_err_x, prev_err_y
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt, dTime
    global prevTime,kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint, sample_time
    global kp_thrust, ki_thrust, kd_thrust

    #-----------------------

    #Assigning PID values here. From symmetry, control for roll and pitch is the same.
    kp_thrust = k_alt[0]
    ki_thrust = k_alt[1]
    kd_thrust = k_alt[2]
    kp_roll = k_roll[0]
    ki_roll = k_roll[1]
    kd_roll = k_roll[2]
    kp_pitch = k_pitch[0]
    ki_pitch = k_pitch[1]
    kd_pitch = k_pitch[2]
    kp_yaw = k_yaw[0]
    ki_yaw = k_yaw[1]
    kd_yaw = k_yaw[2]
    

    setpoint = 0 #this should change according to the desired r,p,y

    if(flag == 0): 
        dTime = 1
        prev_err_x = 0
        prev_err_y = 0
        set_roll = 0
        set_pitch = 0
    
    set_roll, set_pitch = positionControl(roll, pitch, current, target, velocity, dTime)


    #not 100% sure
    err_pitch = pitch - set_pitch
    err_roll = roll - set_roll
    err_yaw = setpoint - yaw
    current_alt_err = req_alt - altitude

    # Publishing error values to be plotted in rqt
    alt_err_pub = rospy.Publisher("/alt_err", Float64, queue_size=10)
    alt_err_pub.publish(current_alt_err)
    roll_err_pub = rospy.Publisher("/roll_err", Float64, queue_size=10)
    roll_err_pub.publish(err_roll)
    pitch_err_pub = rospy.Publisher("/pitch_err", Float64, queue_size=10)
    pitch_err_pub.publish(err_pitch)
    yaw_err_pub = rospy.Publisher("/yaw_err", Float64, queue_size=10)
    yaw_err_pub.publish(err_yaw)
    sample_time = 0.01
    current_time = time.time()

    #Speed found from testing at which drone hovers at a fixed height
    hover_speed = 508.75 
  
    # Flag for checking for the first time the function is called so that values can initilized
    if flag == 0:
        prevTime = 0
        prevErr_roll = 0
        prevErr_pitch = 0
        prevErr_yaw = 0
        pMem_roll = 0
        pMem_pitch = 0
        pMem_yaw = 0
        iMem_roll = 0
        iMem_pitch = 0
        iMem_yaw = 0
        dMem_roll = 0
        dMem_pitch = 0
        dMem_yaw = 0    
        prev_alt_err = 0
        iMem_alt = 0
        dMem_alt = 0
        

    #Define all differential terms
    dTime = current_time - prevTime 
    # print("dTime = ",dTime)
    dErr_alt = current_alt_err - prev_alt_err 
    dErr_pitch = err_pitch - prevErr_pitch
    dErr_roll = err_roll - prevErr_roll
    dErr_yaw = err_yaw - prevErr_yaw
    
    #--------------------------------
    if (dTime >= sample_time):
        # Proportional terms
        pMem_alt = current_alt_err#this is for thrust
        pMem_roll = kp_roll * err_roll
        pMem_pitch = kp_pitch * err_pitch
        pMem_yaw = kp_yaw * err_yaw

        #Integral Terms(e(t))
        iMem_alt += current_alt_err * dTime #this is for thrust
        iMem_roll += err_roll* dTime
        iMem_pitch += err_pitch * dTime
        iMem_yaw += err_yaw * dTime
        #limit integrand values
        if(iMem_alt > 800): iMem_alt = 800
        if(iMem_alt < -800): iMem_alt = -800
        if(iMem_roll > 400): iMem_roll = 400
        if(iMem_roll < -400): iMem_roll = -400
        if(iMem_pitch > 10): iMem_pitch = 10
        if(iMem_pitch < -10): iMem_pitch = -10
        if(iMem_yaw > 400): iMem_yaw = 400
        if(iMem_yaw < -400): iMem_yaw = 400

        #Derivative Terms(e(t))
        dMem_roll = dErr_roll / dTime
        dMem_pitch = dErr_pitch / dTime
        dMem_yaw = dErr_yaw / dTime
        dMem_alt = dErr_alt / dTime
        
        prevTime = current_time

    # Updating all prev terms for next iteration
    prevErr_roll = err_roll
    prevErr_pitch = err_pitch
    prevErr_yaw = err_yaw
    prev_alt_err = current_alt_err


    

    # Final output correction terms after combining PID
    output_alt = kp_thrust*pMem_alt + ki_thrust*iMem_alt + kd_thrust*dMem_alt
    output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
    output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
    output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

    # For Debugging Purposes
    # print("D Time = ",dTime)
    # print("Flag = ",flag)
    # print("P Term Alt= ",pMem_alt)
    # print("I Term Alt= ",iMem_alt)
    # print("D Term Alt= ",dMem_alt)
    print("Altitude Error = ",current_alt_err)
    # print("Altitude Correction = ",output_alt)
    # print("P Term Roll= ",pMem_roll)
    # print("I Term Roll= ",iMem_roll)
    # print("D Term Roll= ",dMem_roll)
    print("Roll Error = ",err_roll)
    # print("Roll Correction = ",output_roll)
    # print("P Term Pitch= ",pMem_pitch)
    # print("I Term Pitch= ",iMem_pitch)
    # print("D Term Pitch= ",dMem_pitch)
    print("Pitch Error = ",err_pitch)
    # print("Pitch Correction = ",output_pitch)
    # print("P Term Yaw= ",pMem_yaw)
    # print("I Term Yaw= ",iMem_yaw)
    # print("D Term Yaw= ",dMem_yaw)
    print("Yaw Error = ",err_yaw)
    # print("Yaw Correction = ",output_yaw)

    # Final thrust
    thrust = hover_speed + output_alt*2.5
    #Limiting thrust
    if(thrust > 1000): 
        thrust = 1000
    elif(thrust < 0):
        thrust = 0    
    print("Thrust = ",thrust)

    speed = prop_speed()
    #uncomment for only altitutde PID testing
    # output_roll=0 
    # output_pitch=0
    # output_yaw=0


    speed.prop1 = (thrust - output_yaw + output_pitch - output_roll)
 
    speed.prop2 = (thrust + output_yaw + output_pitch + output_roll) 
  
    speed.prop3 = (thrust - output_yaw - output_pitch + output_roll) 
 
    speed.prop4 = (thrust + output_yaw - output_pitch - output_roll) 

    #limit the speed
    if(speed.prop1 > 800): speed.prop1 = 800
    if(speed.prop2 > 800): speed.prop2 = 800
    if(speed.prop3 > 800): speed.prop3 = 800
    if(speed.prop4 > 800): speed.prop4 = 800 

    if(speed.prop1 < 10): speed.prop1 = 10
    if(speed.prop2 < 10): speed.prop2 = 10
    if(speed.prop3 < 10): speed.prop3 = 10
    if(speed.prop4 < 10): speed.prop4 = 10 
    rospy.loginfo(speed) 

    return(speed)


#take in the velocity in both the directions
#higher the velocity already is, the closer the output should be towards 0
#Should I write a PID? If so, this is feeding target pitch, roll points


def positionControl(roll, pitch, current, target, velocity, dTime):
    #positive pitch value implies +ve X-direction
    #positive roll value implies -ve Y-direction
    global prev_err_x, prev_err_y

    kp_pitch = 0.001
    ki_pitch = 0.000001
    kd_pitch = 0.01

    kp_roll = 0.001
    ki_roll = 0.000001
    kd_roll = 0.01

    i_term_pitch = 0
    i_term_roll = 0
    c_x, c_y = current[0], current[1]
    t_x, t_y = target[0], target[1]

    vel_x = velocity[0]
    vel_y = velocity[1]

    err_x = c_x - t_x
    err_y = c_y - t_y

    p_term_pitch = kp_pitch * err_x
    i_term_pitch += err_x * dTime 
    d_term_pitch = prev_err_x / dTime 
    if (i_term_pitch > 200): i_term_pitch = 200
    if (i_term_pitch < -200): i_term_pitch = -200

    pitch = -1 * (p_term_pitch + ki_pitch*i_term_pitch + kd_pitch*d_term_pitch)


    p_term_roll = kp_roll * err_y
    i_term_roll += err_y * dTime
    if (i_term_roll > 200): i_term_roll = 200
    if (i_term_roll < -200): i_term_roll = -200
    d_term_roll = prev_err_y / dTime 

    roll = (p_term_roll + ki_roll*i_term_roll + kd_roll*d_term_roll)

    # pitch = (err_x / (abs(t_x) + abs(c_x)) * 1.5) * -1
    # roll = err_y / (abs(t_y) + abs(c_y)) * 2.0

    prev_err_x = err_x
    prev_err_y = err_y

    max = abs(roll) if (abs(roll) > abs(pitch)) else abs(pitch)
    roll = roll / max * 2.5
    pitch = pitch / max * 2.5

    print("Target Roll = ", roll)
    print("Target Pitch = ", pitch)

    # if(roll > 2.5): roll = 2.5
    # if(roll < -2.5): roll = -2.5

    # if(pitch > 2.5): pitch = 2.5
    # if(pitch < -2.5): pitch = -2.5


    #default latitude and longitude values are 19, 72 respectively

    return (roll, pitch)