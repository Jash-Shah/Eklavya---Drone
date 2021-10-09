#! /usr/bin/env python3
import rospy
import time
#import read_sensor_data
from vitarana_drone.msg import prop_speed
from rospy.topics import Publisher
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, Float64MultiArray

def PID_alt(roll, pitch, yaw, req_alt, altitude,flag):
    #global variables are declared to avoid their values resetting to 0
    #global altitude #!!
    global prev_alt_err,i_term,d_term,p_term
    global prevTime,kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint, sample_time
    global kp_thrust, ki_thrust, kd_thrust
    #-----------------------
    #Assigning PID values here. From symmetry, control for roll and pitch is the same.
    kp_thrust = 2
    ki_thrust = 0.002
    kd_thrust = 200 
    kp_roll = 70
    ki_roll = 0.0002
    kd_roll = 89
    kp_pitch = kp_roll
    ki_pitch = ki_roll
    kd_pitch = kd_roll
    kp_yaw = 0.1
    ki_yaw = 0
    kd_yaw = 0
    #print("\nAltitude = " + str(altitude))
    #print("Required alt = ",req_alt)
    # rospy.init_node("pid_alt_node",anonymous=False)
    #rospy.Subscriber("alt_pid", Float64MultiArray, setPID) #!!
    #calVelocity(vel)
    #calImu(imu)

    
    
    # print("roll - control",roll)
    # print("pitch - control",pitch)
    # print("yaw - control",yaw)
    
    # print("kp = ",kp)
    # print("ki = ",ki)
    # print("kd = ",kd)

    setpoint = 0 #this should change according to the desired r,p,y
    # I assume, not 100% sure
    err_pitch = pitch - setpoint
    err_roll = roll - setpoint
    err_yaw = yaw - setpoint
    current_alt_err = req_alt - altitude
    hover_speed = 508.75
    sample_time = 0
    current_time = time.time()
    # print("Current time = ",current_time)
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
        i_term = 0
        d_term = 0

    #Define the difference in error or dErr
    print("Prev Time = ",prevTime)
    dTime = current_time - prevTime 
    dErr_alt = current_alt_err - prev_alt_err #difference in error
    dErr_pitch = err_pitch - prevErr_pitch
    dErr_roll = err_roll - prevErr_roll
    dErr_yaw = err_yaw - prevErr_yaw

    #--------------------------------
    if (dTime >= sample_time):
        p_term = current_alt_err#this is for thrust
        if flag == 0:

            flag += 1
        else:
            #proportional(e(t))
            pMem_roll = kp_roll * err_roll
            pMem_pitch = kp_pitch * err_pitch
            pMem_yaw = kp_yaw * err_yaw
            #integral(e(t))
            i_term += current_alt_err * dTime #this is for thrust
            iMem_roll += err_pitch * dTime
            iMem_pitch += err_roll * dTime
            iMem_yaw += err_yaw * dTime
            #derivative(e(t))
            dMem_roll = dErr_roll / dTime
            dMem_pitch = dErr_pitch / dTime
            dMem_yaw = dErr_yaw / dTime
            d_term =  dErr_alt/dTime

    prevTime = current_time
    # print("Prev Time = ",prevTime)
    prevErr_roll = err_roll
    prevErr_pitch = err_pitch
    prevErr_yaw = err_yaw
    prev_alt_err = current_alt_err

    output_alt = kp_thrust*p_term + ki_thrust*i_term + kd_thrust*d_term
    output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
    output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
    output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

    print("Altitude Correction = ",output_alt)
    print("Flag = ",flag)
    print("D Time = ",dTime)
    print("P Term = ",p_term)
    print("I Term = ",i_term)
    print("D Term = ",d_term)
    thrust = hover_speed + output_alt*1.2


    #we need to limit this thrust
    if(thrust > 1000): 
        thrust = 1000
    elif(thrust < 0):
        thrust = 0
    
    print("Thrust = ",thrust)

    speed = prop_speed()
    # speed.prop1 = thrust
    # speed.prop2 = thrust
    # speed.prop3 = thrust
    # speed.prop4 = thrust

    #values coming out are strange
    #Need to fine tune a lot

    #uncomment for only altitutde PID testing
    output_roll=0
    output_pitch=0
    output_yaw=0


    speed.prop1 = (thrust - output_yaw + output_pitch - output_roll) 
 
    speed.prop2 = (thrust + output_yaw + output_pitch + output_roll) 
  
    speed.prop3 = (thrust - output_yaw - output_pitch + output_roll) 
 
    speed.prop4 = (thrust + output_yaw - output_pitch - output_roll) 

    #limit the speed
    if(speed.prop1 > 1000): speed.prop1 = 1000
    if(speed.prop2 > 1000): speed.prop2 = 1000
    if(speed.prop3 > 1000): speed.prop3 = 1000
    if(speed.prop4 > 1000): speed.prop4 = 1000 

    if(speed.prop1 < 0): speed.prop1 = 0
    if(speed.prop2 < 0): speed.prop2 = 0
    if(speed.prop3 < 0): speed.prop3 = 0
    if(speed.prop4 < 0): speed.prop4 = 0 
    rospy.loginfo(speed) #!!
    return(speed)
    #message_pub.publish(speed)