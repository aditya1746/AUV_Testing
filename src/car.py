#! /usr/bin/env python
from numpy import float32
import rospy
from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
# from auv_codes2.cfg import PID_yawConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
# from auv_codes2.msg import pid_straight_msg
import time
from matplotlib import pyplot as plt
import signal, sys

#    FL BL FR BR ML MR
# F  C  C  C  C 
# B  A  A  A  A
# L  A  C  C  A
# R  C  A  A  C
# D  -  -  -  -  C  C
# U  -  -  -  -  A  A

xlist, x = [], 0
yawErr = []
y1, y2, y3, y4 = [], [], [], []

direction_desired =0 # 0- straight, 1 -left, 2-right, 3- back
desired_speed=0   # 0- 0, 1 - 50, 2- 100, 3- 150 , 4 -200
pwmBase_l, pwmBase_r = 1480, 1480
yaw_desired, cnt = 0, 0
yaw = 0
acc_y = 0
prev_err_y, int_err_y, prev_err_ay, int_err_ay, prev_err_ax, int_err_ax, prev_err_dist_rt, int_err_dist_rt = 0,0,0,0,0,0,0,0
prev_err_od, int_err_od= 0, 0


KP_y, KD_y, KI_y = 8.0, 1.0,0.0

KP_od, KD_od, KI_od = 0.0, 0.0, 0.0
KP_s, KD_s, KI_s = 0.0, 0.0, 0.0
KP_ay, KD_ay, KI_ay = 0.0,0.0,0.0
KP_ax, KD_ax, KI_ax = 0.0,0.0,0.0

desired_dist_rt, dist_rt, con_rt = -17, -1, -1
err_y_thresh = 4 #degrees
err_a_thresh = 1
MAXpwm, MINpwm = 300, -300
startpwm = 40
pwm_fr, pwm_fl, pwm_br, pwm_bl = startpwm, startpwm, startpwm, startpwm  # ================== for going straight
# pwm_fr, pwm_fl, pwm_br, pwm_bl = -startpwm, startpwm, startpwm, -startpwm  # ================== for going right

correction_y, correction_ax, correction_ay = 0.0, 0.0, 0.0

# x-axis ^ 
# y-axis <
# err_y(aw) is positive => robot is turned towards left, when we are looking from back
# err_ay is positive => robot is moving in +y dirn

def sigint_handler(signal, frame):
	print("Yeeeeeeee, function called !!!!!!!!!!!!!!")
	stopCar()
	sys.exit(0)

def straightLine_pid_imu():
	global prev_err_y, int_err_y, prev_err_ay, int_err_ay, prev_err_dist_rt, int_err_dist_rt, prev_err_od, int_err_od

	global pwm_fr, pwm_fl, pwm_br, pwm_bl
	global xlist, x, yawErr, y1, y2, y3, y4

	# print(KP_y, KD_y, KI_y)

	# pwm_fr, pwm_br, pwm_fl, pwm_bl = 100, 100, 100, 100 # ================== 100 basically represent base velocities
	
	err_y = yaw - yaw_desired
	diff_err_y = err_y - prev_err_y
	int_err_y += err_y 
	prev_err_y = err_y
        
	correction_y = KP_y*err_y + KD_y*diff_err_y + KI_y*int_err_y

    err_od = object_d
	diff_err_od = err_od - prev_err_od
	int_err_od += err_od 
	prev_err_od = err_od
        
	correction_od = KP_od*err_od + KD_od*diff_err_od + KI_od*int_err_od

	print("yaw")
	print(err_y, correction_y)    

	err_ay = acc_y
	diff_err_ay = err_ay - prev_err_ay
	int_err_ay += err_ay 
	prev_err_ay = err_ay
        
	correction_ay = KP_ay*err_ay + KD_ay*diff_err_ay + KI_ay*err_ay

	err_dist_rt = dist_rt - desired_dist_rt
	diff_err_dist_rt = err_dist_rt - prev_err_dist_rt
	int_err_dist_rt += err_dist_rt
	prev_err_dist_rt = err_dist_rt

	correction_dist_rt = KP_s*err_dist_rt + KD_s*diff_err_dist_rt + KI_s*int_err_dist_rt

	# print("acceleration")
	# print(err_ay, correction_ay) 

    base=0 
    # specifying the base speed


    if(desired_speed==0):
        base=0
    if(desired_speed==1):
        base=50
    if(desired_speed==2):
        base=100
    if(desired_speed==3):
        base=150
    if(desired_speed==4):
        base=200

    if(direction_desired==0):
	    st_fr, st_fl, st_br, st_bl = base, base, base, base

    if(direction_desired==1):
	    st_fr, st_fl, st_br, st_bl = base, -1*base, -1*base, base
    if(direction_desired==2):
	    st_fr, st_fl, st_br, st_bl = -1*base, base, base, -1*base

    if(direction_desired==3):
	    st_fr, st_fl, st_br, st_bl = -1*base, -1*base, -1*base, -1*base
	# rt_fr, rt_fl, rt_br, rt_bl = -50, 50, 50, -50
	# lt_fr, lt_fl, lt_br, lt_bl = 50, -50, -50, 50

	pwm_fr, pwm_fl, pwm_br, pwm_bl = st_fr, st_fl, st_br, st_bl
	# pwm_fr, pwm_fl, pwm_br, pwm_bl = rt_fr, rt_fl, rt_br, rt_bl
	# pwm_fr, pwm_fl, pwm_br, pwm_bl = lt_fr, lt_fl, lt_br, lt_bl
	

    if(object_d==0):
        pwm_fr -= correction_y # decreasing the force of front right, if err is +ve
        pwm_fl += correction_y # increasing the force of front left, if err is +ve
        pwm_br -= correction_y
        pwm_bl += correction_y
    else:
        pwm_fr -= correction_od # decreasing the force of front right, if err is +ve
        pwm_fl += correction_od # increasing the force of front left, if err is +ve
        pwm_br -= correction_od
        pwm_bl += correction_od

	pwm_fl += correction_ay
	pwm_br += correction_ay
	pwm_fr -= correction_ay
	pwm_br -= correction_ay

	pwm_fr -= correction_dist_rt # decreasing the force of front right, if err is +ve, +ve err=> right me jana chahiye
	pwm_fl += correction_dist_rt # increasing the force of front left, if err is +ve
	pwm_br += correction_dist_rt
	pwm_bl -= correction_dist_rt

	pwm_fr = max(pwm_fr, MINpwm)
	pwm_fl = max(pwm_fl, MINpwm)
	pwm_br = max(pwm_br, MINpwm)
	pwm_bl = max(pwm_bl, MINpwm)
	pwm_fr = min(pwm_fr, MAXpwm)
	pwm_fl = min(pwm_fl, MAXpwm)
	pwm_br = min(pwm_br, MAXpwm)
	pwm_bl = min(pwm_bl, MAXpwm)

	print(pwm_fr, pwm_fl, pwm_br, pwm_bl)

	if(pwm_fr<=MAXpwm and pwm_fl<=MAXpwm and pwm_br<=MAXpwm and pwm_bl<=MAXpwm):
		fr, br, fl, bl = pwmBase_r - 1*int(pwm_fr), pwmBase_r - int(pwm_br),pwmBase_l + int(pwm_fl), pwmBase_l + int(pwm_bl)
		pwm_msg = str(fr) + ' ' + str(fl) + ' ' + str(br) + ' ' + str(bl) + ' '
		pub.publish(pwm_msg)
	# rospy.loginfo(yaw)

	# xlist.append(x)
	# x = x+1
	# y1.append(pwm_fr)
	# y2.append(pwm_fl)
	# y3.append(pwm_br)
	# y4.append(pwm_bl)
	# yawErr.append(err_y)

	# plt.plot(xlist, yawErr, label = 'Err in yaw')
	# plt.show()




	




def yaw_callback(msg):
	global yaw, yaw_desired, cnt
	yaw = msg.data

	if(yaw < -360):
		yaw = yaw%360 + 360
	if(yaw > 360):
		yaw = yaw%360
	# if(cnt==0):
	# 	yaw_desired=yaw
	# 	cnt = 1
	
	straightLine_pid_imu()
	# startCar()
	# right_pid_imu()


	
	# straightLine_pid_imu()
	




def dist_r_cb(msg):

	global desired_dist_rt, dist_rt

	if(desired_dist_rt == -17):
		desired_dist_rt = msg.data

	dist_rt = msg.data

def con_r_cb(msg):

	global con_rt
	con_rt = msg.data

def yaw_d_cb(msg):

	global yaw_desired
	yaw_desired = msg.data

def direction_cb(msg):

	global direction_desired
	direction_desired = msg.data

def speed_cb(msg):

	global desired_speed
	desired_speed = msg.data

def od_cb(msg):

	global object_d
	object_d = msg.data


if __name__ == "__main__":
	
	
	rospy.init_node("car", anonymous = False)
	q = 10000

	pub=rospy.Publisher("PWM_VALUE_car",String ,queue_size=q)
	rospy.Subscriber("angle_z", Float64, yaw_callback, queue_size=1000)
	
	rospy.Subscriber("dist_r", Float64, dist_r_cb)
	rospy.Subscriber("con_r", Float64, con_r_cb)
    rospy.Subscriber("yaw_d", Float64, yaw_d_cb)
    rospy.Subscriber("direction", Float64, direction_cb)
    rospy.Subscriber("speed", Float64, speed_cb)
    rospy.Subscriber("object_distance", Float64, od_cb)



	rate = rospy.Rate(1)



	rospy.spin()

