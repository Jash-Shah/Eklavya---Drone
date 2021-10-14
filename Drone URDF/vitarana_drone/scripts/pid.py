# #! /usr/bin/env python3
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


def PID_alt(roll, pitch, yaw,x,y, req_alt, altitude, k_alt, k_roll, k_pitch, k_yaw, k_x, k_y, velocity, k_vel, flag): 
    #global variables are declared to avoid their values resetting to 0
    global prev_alt_err,iMem_alt,dMem_alt,pMem_alt
    global prevTime,kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint_roll,setpoint_pitch, sample_time,current_time
    global kp_x,ki_x,kd_x,kp_y,ki_y,kd_y,target_x,target_y
    global kp_thrust, ki_thrust, kd_thrust

    #--------------------------------------------------------------------

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
    kp_x = k_x[0]
    ki_x = k_x[1]
    kd_x = k_x[2]
    kp_y = k_y[0]
    ki_y = k_y[1]
    kd_y = k_y[2]
    setpoint_roll = 0  #this should change according to the desired r,p,y
    setpoint_pitch = 0  #this should change according to the desired r,p,y
    target_x = 1
    target_y = 0
    sample_time = 0.005

    current_time = time.time()

    #WE NEED TO ACTIVELY CORRECT VELOCITY
    position_controller(target_x, target_y, x, y, velocity, k_vel, flag)

    #100% sure
    print("Setpoint pitch = ",setpoint_pitch)
    print("Setpoint roll = ",setpoint_roll)

    err_pitch = pitch - setpoint_pitch
    err_roll = roll - 0
    err_yaw = 0 - yaw
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
    #print("dTime = ",dTime)
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
        if(iMem_alt <-800): iMem_alt = -800
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
    # print("Altitude Error = ",current_alt_err)
    # print("Altitude Correction = ",output_alt)
    # print("P Term Roll= ",pMem_roll)
    # print("I Term Roll= ",iMem_roll)
    # print("D Term Roll= ",dMem_roll)
    # print("Roll Error = ",err_roll)
    # print("Roll Correction = ",output_roll)
    # print("P Term Pitch= ",pMem_pitch)
    # print("I Term Pitch= ",iMem_pitch)
    # print("D Term Pitch= ",dMem_pitch)
    # print("Pitch Error = ",err_pitch)
    # print("Pitch Correction = ",output_pitch)
    # print("P Term Yaw= ",pMem_yaw)
    # print("I Term Yaw= ",iMem_yaw)
    # print("D Term Yaw= ",dMem_yaw)
    # print("Yaw Error = ",err_yaw)
    # print("Yaw Correction = ",output_yaw)

    # Final thrust
    thrust = hover_speed + output_alt*2.5
    #Limiting thrust
    if(thrust > 800): 
        thrust = 800
    elif(thrust < 10):
        thrust = 10    
    # print("Thrust = ",thrust)

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


def position_controller(target_x, target_y, x, y, velocity, k_vel, flag):
    global current_time,prevTime,dTime
    global prevErr_x,prevErr_y,pMem_x,pMem_y,iMem_x,iMem_y,dMem_x,dMem_y, prevErr_vel_x, prevErr_vel_y
    global pMem_vel_x, iMem_vel_x, dMem_vel_x
    global pMem_vel_y, iMem_vel_y, dMem_vel_y
    global kp_x,ki_x,kd_x
    global kp_y,ki_y,kd_y
    global setpoint_pitch,setpoint_roll

    # print("Kp x = ",kp_x)
    # print("Kp y = ",kp_y)
    vel_x = velocity[0]
    vel_y = velocity[1]

    
    kp_vel_x = 0.05 #k_vel[0]
    ki_vel_x = 0.003#k_vel[1]
    kd_vel_x = 0    #k_vel[2]

    kp_vel_y = 0.05 #k_vel[3]
    ki_vel_y = 0.003#k_vel[4]
    kd_vel_y = 0    #k_vel[5]
    

    err_x = x - target_x
    err_y = y - target_y
    err_vel_x = vel_x - 0
    err_vel_y = vel_y - 0

    print("Vel X Error = ", err_vel_x)
    print("X Error = ",err_x)
    print("Y Error = ",err_y)
    x_err_pub = rospy.Publisher("/x_err", Float64, queue_size=10)
    x_err_pub.publish(err_x)
    y_err_pub = rospy.Publisher("/y_err", Float64, queue_size=10)
    y_err_pub.publish(err_y)
    vel_x_err_pub = rospy.Publisher("/vel_x_err", Float64, queue_size=10)
    vel_x_err_pub.publish(err_vel_x)
    if (flag==0):
        prevTime = 0
        prevErr_x = 0
        prevErr_y = 0
        prevErr_vel_x = 0
        prevErr_vel_y = 0
        pMem_vel_x = 0
        pMem_vel_y = 0
        pMem_x = 0
        pMem_y = 0
        iMem_vel_x = 0
        iMem_vel_y = 0
        iMem_x = 0
        iMem_y = 0
        dMem_vel_x = 0
        dMem_vel_y = 0
        dMem_x = 0
        dMem_y = 0
    
    dTime = current_time - prevTime
    dErr_x = err_x - prevErr_x
    dErr_y = err_y - prevErr_y
    dErr_vel_x = err_vel_x - prevErr_vel_x
    dErr_vel_y = err_vel_y - prevErr_vel_y

    if(dTime >=sample_time):
        pMem_x = kp_x*err_x
        pMem_y = kp_y*err_y
        pMem_vel_x = kp_vel_x*err_x
        pMem_vel_y = kp_vel_y*err_y


        iMem_x += err_x*dTime
        iMem_y += err_y*dTime
        iMem_vel_x += err_vel_x*dTime
        iMem_vel_y += err_vel_y*dTime

        if(iMem_vel_x>10000): iMem_vel_x = 10
        if(iMem_vel_x<-10): iMem_vel_x=-10
        if(iMem_vel_y>10): iMem_vel_y = 10
        if(iMem_vel_y<-10): iMem_vel_y=-10
        if(iMem_x>10): iMem_x = 10
        if(iMem_x<-10): iMem_x=-10

        dMem_x = dErr_x/dTime
        dMem_y = dErr_y/dTime
        dMem_vel_x = dErr_vel_x/dTime
        dMem_vel_y = dErr_vel_y/dTime

    prevErr_x = err_x
    prevErr_y = err_y
    #print("P Term X = ",pMem_x)
    #print("I Term X = ",iMem_x)
    #print("D Term X = ",dMem_x)

    print("P Term Vel X = ",pMem_vel_x)
    print("I Term Vel X = ",iMem_vel_x)
    print("D Term Vel X = ",dMem_vel_x)

    output_x = pMem_x + ki_x*iMem_x + kd_x*dMem_x
    output_y = pMem_y + ki_y*iMem_y + kd_y*dMem_y
    output_vel_x = pMem_vel_x + ki_vel_x*iMem_vel_x + kd_vel_x*dMem_vel_x
    #not commented these lines, but output roll is commented 
    output_vel_y = pMem_vel_y + ki_vel_y*iMem_vel_y + kd_vel_y*dMem_vel_y


    #If error in x is greater than 1 then try to set velocity 0 first
    #Then start correcting for other things
    # lim_vel = 1
    # limit = False
    # if(vel_x > lim_vel): limit = True
    # if (vel_x < -lim_vel): limit = True

    # if(limit): output_x = 0


    if(output_x>5):
        output_x = 5
        print('MAXIMUM HIT')
    if(output_x<-5):
        output_x = -5
        print('MINIMUM HIT')

    if(abs(err_x) > 2 and abs(vel_x) < 1.5):
        setpoint_pitch = -(output_x)
    else:
        setpoint_pitch = -(output_vel_x)


    #setpoint_pitch = err_x + dErr_x

    # if(abs(err_y) > 2 or abs(vel_y) < 5):
    #     setpoint_roll = output_y
    # else:
    #     setpoint_roll = output_vel_y

    setpoint_roll = output_y


# import rospy
# import time
# from vitarana_drone.msg import prop_speed
# from rospy.topics import Publisher
# from sensor_msgs.msg import NavSatFix
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Vector3Stamped
# from geometry_msgs.msg import Vector3
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from std_msgs.msg import Float64, Float64MultiArray


# def PID_alt(roll, pitch, yaw,x,y, req_alt, altitude, k_alt, k_roll, k_pitch, k_yaw,k_x,k_y, flag): 
#     #global variables are declared to avoid their values resetting to 0
#     global prev_alt_err,iMem_alt,dMem_alt,pMem_alt
#     global prevTime,kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw,kp_x,ki_x,kd_x,kp_y,ki_y,kd_y, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw,pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, setpoint, sample_time,setpos
#     global pMem_x,pMem_y,iMem_x,iMem_y,dMem_x,dMem_y,prevErr_x,prevErr_y
#     global kp_thrust, ki_thrust, kd_thrust

#     #-----------------------

#     #Assigning PID values here. From symmetry, control for roll and pitch is the same.
#     kp_thrust = k_alt[0]
#     ki_thrust = k_alt[1]
#     kd_thrust = k_alt[2]
#     kp_roll = k_roll[0]
#     ki_roll = k_roll[1]
#     kd_roll = k_roll[2]
#     kp_pitch = k_pitch[0]
#     ki_pitch = k_pitch[1]
#     kd_pitch = k_pitch[2]
#     kp_yaw = k_yaw[0]
#     ki_yaw = k_yaw[1]
#     kd_yaw = k_yaw[2]
#     kp_x = k_x[0]
#     ki_x = k_x[1]
#     kd_x = k_x[2]
#     kp_y = k_y[0]
#     ki_y = k_y[1]
#     kd_y = k_y[2]
#     setpoint = 0 #this should change according to the desired r,p,y
#     setpos = 0

#     #100% sure
#     err_pitch = - pitch + setpoint
#     err_roll = roll - setpoint
#     err_yaw = setpoint - yaw
#     err_x = x - setpos
#     err_y = y - setpos
#     current_alt_err = req_alt - altitude

#     # Publishing error values to be plotted in rqt
#     alt_err_pub = rospy.Publisher("/alt_err", Float64, queue_size=10)
#     alt_err_pub.publish(current_alt_err)
#     roll_err_pub = rospy.Publisher("/roll_err", Float64, queue_size=10)
#     roll_err_pub.publish(err_roll)
#     pitch_err_pub = rospy.Publisher("/pitch_err", Float64, queue_size=10)
#     pitch_err_pub.publish(err_pitch)
#     yaw_err_pub = rospy.Publisher("/yaw_err", Float64, queue_size=10)
#     yaw_err_pub.publish(err_yaw)
#     x_err_pub = rospy.Publisher("/x_err", Float64, queue_size=10)
#     x_err_pub.publish(err_x)
#     y_err_pub = rospy.Publisher("/y_err", Float64, queue_size=10)
#     y_err_pub.publish(err_y)

#     sample_time = 0.01
#     current_time = time.time()

#     #Speed found from testing at which drone hovers at a fixed height
#     hover_speed = 508.75 
  
#     # Flag for checking for the first time the function is called so that values can initilized
#     if flag == 0:
#         prevTime = 0
#         prevErr_roll = 0
#         prevErr_pitch = 0
#         prevErr_yaw = 0
#         prevErr_x = 0
#         prevErr_y = 0
#         pMem_roll = 0
#         pMem_pitch = 0
#         pMem_yaw = 0
#         pMem_x = 0
#         pMem_y = 0
#         iMem_roll = 0
#         iMem_pitch = 0
#         iMem_yaw = 0
#         iMem_x = 0
#         iMem_y = 0
#         dMem_roll = 0
#         dMem_pitch = 0
#         dMem_yaw = 0
#         dMem_x = 0    
#         dMem_y = 0    
#         prev_alt_err = 0
#         iMem_alt = 0
#         dMem_alt = 0

#     #Define all differential terms
#     dTime = current_time - prevTime 
#     print("dTime = ",dTime)
#     dErr_alt = current_alt_err - prev_alt_err 
#     dErr_pitch = err_pitch - prevErr_pitch
#     dErr_roll = err_roll - prevErr_roll
#     dErr_yaw = err_yaw - prevErr_yaw
#     dErr_x = err_x - prevErr_x
#     dErr_y = err_y - prevErr_y

#     #--------------------------------
#     if (dTime >= sample_time):
#         # Proportional terms
#         pMem_x = kp_x * err_x
#         pMem_y = kp_y * err_y
#         pMem_alt = current_alt_err#this is for thrust
#         pMem_roll = kp_roll * err_roll
#         pMem_pitch = kp_pitch * err_pitch
#         pMem_yaw = kp_yaw * err_yaw

#         #Integral Terms(e(t))
#         iMem_x += err_x * dTime
#         iMem_y += err_y * dTime
#         iMem_alt += current_alt_err * dTime #this is for thrust
#         iMem_roll += err_roll* dTime
#         iMem_pitch += err_pitch * dTime
#         iMem_yaw += err_yaw * dTime
#         #limit integrand values
#         if(iMem_alt > 800): iMem_alt = 800
#         if(iMem_alt < -800): iMem_alt = -800
#         if(iMem_x > 800): iMem_x = 800
#         if(iMem_x < -800): iMem_x = -800
#         if(iMem_y > 800): iMem_y = 800
#         if(iMem_y < -800): iMem_y = -800
#         if(iMem_roll > 400): iMem_roll = 400
#         if(iMem_roll < -400): iMem_roll = -400
#         if(iMem_pitch > 400): iMem_pitch = 400
#         if(iMem_pitch < -400): iMem_pitch = -400
#         if(iMem_yaw > 400): iMem_yaw = 400
#         if(iMem_yaw < -400): iMem_yaw = 400

#         #Derivative Terms(e(t))
#         dMem_roll = dErr_roll / dTime
#         dMem_pitch = dErr_pitch / dTime
#         dMem_yaw = dErr_yaw / dTime
#         dMem_x = dErr_x/dTime
#         dMem_y = dErr_y/dTime
#         dMem_alt = dErr_alt / dTime
        
#         prevTime = current_time

#     # Updating all prev terms for next iteration
#     prevErr_roll = err_roll
#     prevErr_pitch = err_pitch
#     prevErr_yaw = err_yaw
#     prevErr_x = err_x
#     prevErr_y = err_y
#     prev_alt_err = current_alt_err

#     # Final output correction terms after combining PID
#     output_x = kp_x*pMem_x + ki_x*iMem_x + kd_x*dMem_x
#     output_y = kp_y*pMem_y + ki_y*iMem_y + kd_y*dMem_y
#     output_alt = kp_thrust*pMem_alt + ki_thrust*iMem_alt + kd_thrust*dMem_alt
#     output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
#     output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
#     output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

#     # For Debugging Purposes
#     # print("D Time = ",dTime)
#     # print("Flag = ",flag)
#     # print("P Term Alt= ",pMem_alt)
#     # print("I Term Alt= ",iMem_alt)
#     # print("D Term Alt= ",dMem_alt)
#     print("Altitude Error = ",current_alt_err)
#     # print("Altitude Correction = ",output_alt)
#     # print("P Term Roll= ",pMem_roll)
#     # print("I Term Roll= ",iMem_roll)
#     # print("D Term Roll= ",dMem_roll)
#     print("Roll Error = ",err_roll)
#     # print("Roll Correction = ",output_roll)
#     # print("P Term Pitch= ",pMem_pitch)
#     # print("I Term Pitch= ",iMem_pitch)
#     # print("D Term Pitch= ",dMem_pitch)
#     print("Pitch Error = ",err_pitch)
#     # print("Pitch Correction = ",output_pitch)
#     # print("P Term Yaw= ",pMem_yaw)
#     # print("I Term Yaw= ",iMem_yaw)
#     # print("D Term Yaw= ",dMem_yaw)
#     print("Yaw Error = ",err_yaw)
#     # print("Yaw Correction = ",output_yaw)
#     # print("P Term Y= ",pMem_y)
#     # print("I Term Y= ",iMem_y)
#     # print("D Term Y= ",dMem_y)
#     print("Y Error = ",err_y)
#     # print("Y Correction = ",output_y)
#     # print("P Term X= ",pMem_x)
#     # print("I Term X= ",iMem_x)
#     # print("D Term X= ",dMem_x)
#     print("X Error = ",err_x)
#     # print("X Correction = ",output_x)

#     # Final thrust
#     thrust = hover_speed + output_alt*2.5
#     #Limiting thrust
#     if(thrust > 1000): 
#         thrust = 1000
#     elif(thrust < 0):
#         thrust = 0    
#     print("Thrust = ",thrust)

#     speed = prop_speed()
#     #uncomment for only altitutde PID testing
#     # output_roll=0 
#     # output_pitch=0
#     # output_yaw=0


#     speed.prop1 = (thrust - output_yaw + output_pitch - output_roll) 
 
#     speed.prop2 = (thrust + output_yaw + output_pitch + output_roll) 
  
#     speed.prop3 = (thrust - output_yaw - output_pitch + output_roll) 
 
#     speed.prop4 = (thrust + output_yaw - output_pitch - output_roll) 

#     #limit the speed
#     if(speed.prop1 > 1000): speed.prop1 = 1000
#     if(speed.prop2 > 1000): speed.prop2 = 1000
#     if(speed.prop3 > 1000): speed.prop3 = 1000
#     if(speed.prop4 > 1000): speed.prop4 = 1000 

#     if(speed.prop1 < 0): speed.prop1 = 0
#     if(speed.prop2 < 0): speed.prop2 = 0
#     if(speed.prop3 < 0): speed.prop3 = 0
#     if(speed.prop4 < 0): speed.prop4 = 0 
#     rospy.loginfo(speed) 

#     return(speed)





# #------------------
# # suppose current_x = 0.0
# # required_x = 10.0
# # err_x = -10.0
# #                        |                     
# #             p1       |        p2
# # y <-----         base
# #             p4               p3        
# # position_controller(yaw,target_posn,x,y) --> roll,pitch-->setpoint
# # {
# #     err_x = x - target_x
# #     err_y  =y-target_y
# #     If error in x is -ve then pitch +ve(pitch forward)
# #     the angle of pitch is dependent on err_x(and it should be limited (30 degrees for now))
# #     pitch = err_x

# # } 