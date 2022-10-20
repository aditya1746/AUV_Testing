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

pwmBase_l, pwmBase_r = 1480, 1480
yaw_desired, cnt = 0, 0
yaw = 0
acc_y = 0
prev_err_y, int_err_y, prev_err_ay, int_err_ay, prev_err_ax, int_err_ax, prev_err_dist_rt, int_err_dist_rt = 0,0,0,0,0,0,0,0

KP_y, KD_y, KI_y = 8.0, 1.0,0.0
KP_s, KD_s, KI_s = 0.5, 0.0, 0.0
KP_ay, KD_ay, KI_ay = 0.0,0.0,0.0
KP_ax, KD_ax, KI_ax = 0.0,0.0,0.0

desired_dist_rt, dist_rt, con_rt = -17, -1, -1
err_y_thresh = 3 #degrees
err_a_thresh = 1
MAXpwm, MINpwm = 350, -350
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
	global prev_err_y, int_err_y, prev_err_ay, int_err_ay, prev_err_dist_rt, int_err_dist_rt
	global pwm_fr, pwm_fl, pwm_br, pwm_bl
	global xlist, x, yawErr, y1, y2, y3, y4

	# print(KP_y, KD_y, KI_y)

	# pwm_fr, pwm_br, pwm_fl, pwm_bl = 100, 100, 100, 100 # ================== 100 basically represent base velocities
	
	err_y = yaw - yaw_desired
	diff_err_y = err_y - prev_err_y
	int_err_y += err_y 
	prev_err_y = err_y
        
	correction_y = KP_y*err_y + KD_y*diff_err_y + KI_y*int_err_y

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

	st_fr, st_fl, st_br, st_bl = 50, 50, 50, 50
	rt_fr, rt_fl, rt_br, rt_bl = -50, 50, 50, -50
	lt_fr, lt_fl, lt_br, lt_bl = 50, -50, -50, 50

	pwm_fr, pwm_fl, pwm_br, pwm_bl = st_fr, st_fl, st_br, st_bl
	# pwm_fr, pwm_fl, pwm_br, pwm_bl = rt_fr, rt_fl, rt_br, rt_bl
	# pwm_fr, pwm_fl, pwm_br, pwm_bl = lt_fr, lt_fl, lt_br, lt_bl
	
	pwm_fr -= correction_y # decreasing the force of front right, if err is +ve
	pwm_fl += correction_y # increasing the force of front left, if err is +ve
	pwm_br -= correction_y
	pwm_bl += correction_y

	pwm_fl += correction_ay
	pwm_br += correction_ay
	pwm_fr -= correction_ay
	pwm_br -= correction_ay

	if(err_y < abs(err_y_thresh) and con_rt >= 75.0):
		pwm_fr -= correction_dist_rt # decreasing the force of front right, if err is +ve, +ve err=> right me jana chahiye
		pwm_fl += correction_dist_rt # increasing the force of front left, if err is +ve
		pwm_br += correction_dist_rt
		pwm_bl -= correction_dist_rt

		print("drift")
		print(err_dist_rt, correction_dist_rt)  

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

def right_pid_imu():
	global prev_err_y, int_err_y, prev_err_ax, int_err_ax
	global pwm_fr, pwm_fl, pwm_br, pwm_bl
	global xlist, x, yawErr, y1, y2, y3, y4

	# pwm_fr, pwm_br, pwm_fl, pwm_bl = 100, 100, 100, 100 # ================== 100 basically represent base velocities
	
	err_y = yaw - yaw_desired
	diff_err_y = err_y - prev_err_y
	int_err_y += err_y 
	prev_err_y = err_y
        
	correction_y = KP_y*err_y + KD_y*diff_err_y + KI_y*int_err_y

	print("yaw")
	print(err_y, correction_y)    

	err_ax = acc_x
	diff_err_ax = err_ax - prev_err_ax
	int_err_ax += err_ax 
	prev_err_ax = err_ax
        
	correction_ax = KP_ax*err_ax + KD_ax*diff_err_ax + KI_ax*err_ax

	# print("acceleration")
	# print(err_ax, correction_ax)    
	
	pwm_fr -= correction_y # increasing the force of front right
	pwm_br -= correction_y # decreasing the force of back right

	# err_ax is +ve => robot going backwards
	pwm_fl += correction_ax # increasing the force of front left and back right, decreasing the force of front right and back right
	pwm_br += correction_ax
	pwm_fr += correction_ax
	pwm_br += correction_ax

	pwm_fr = min(pwm_fr, -startpwm)
	pwm_fl = max(pwm_fl, startpwm)
	pwm_br = max(pwm_br, startpwm)
	pwm_bl = min(pwm_bl, -startpwm)
	pwm_fr = max(pwm_fr, MINpwm)
	pwm_fl = min(pwm_fl, MAXpwm)
	pwm_br = min(pwm_br, MAXpwm)
	pwm_bl = max(pwm_bl, MINpwm)

	print(pwm_fr, pwm_fl, pwm_br, pwm_bl)

	if(pwm_fr<=MAXpwm and pwm_fl<=MAXpwm and pwm_br<=MAXpwm and pwm_bl<=MAXpwm):
		fr, br, fl, bl = pwmBase_r - 1*int(pwm_fr), pwmBase_r - int(pwm_br),pwmBase_l + int(pwm_fl), pwmBase_l + int(pwm_bl)
		pwm_msg = str(fr) + ' ' + str(fl) + ' ' + str(br) + ' ' + str(bl) + ' '
		pub.publish(pwm_msg)
	# rospy.loginfo(yaw)

	# x = x+1
	# y1.append(pwm_fr)
	# y2.append(pwm_fl)
	# y3.append(pwm_br)
	# y4.append(pwm_bl)
	# yawErr.append(err_y)

	# plt.plot(xlist, yawErr, label = 'Err in yaw')
	# plt.show()

def stopCar():
	global pwm_fr, pwm_br, pwm_fl, pwm_bl 
	pwm_fr, pwm_br, pwm_fl, pwm_bl = pwmBase_r, pwmBase_r, pwmBase_l, pwmBase_l
	pwm_msg = str(pwm_fr) + ' ' + str(pwm_fl) + ' ' + str(pwm_br) + ' ' + str(pwm_bl) + ' '		
	pub.publish(pwm_msg)
	print ("function called")
	
def startCar():
	global pwm_fr, pwm_br, pwm_fl, pwm_bl 
	pwm_fr, pwm_br, pwm_fl, pwm_bl = startpwm, startpwm, startpwm, startpwm
	fr, br, fl, bl = pwmBase_r - 1*int(pwm_fr), pwmBase_r - int(pwm_br),pwmBase_l + int(pwm_fl), pwmBase_l + int(pwm_bl)
	pwm_msg = str(fr) + ' ' + str(fl) + ' ' + str(br) + ' ' + str(bl) + ' '		
	pub.publish(pwm_msg)
	
def accy_callback(msg):
	global acc_y
	acc_y = msg.data

def accx_callback(msg):
	global acc_x
	acc_x = msg.data

def yaw_callback(msg):
	global yaw, yaw_desired, cnt
	yaw = msg.data

	if(yaw < -360):
		yaw = yaw%360 + 360
	if(yaw > 360):
		yaw = yaw%360
	if(cnt==0):
		yaw_desired=yaw
		cnt = 1
	
	straightLine_pid_imu()
	# startCar()
	# right_pid_imu()

def yawCBinGUI(msg):
	global yaw, yaw_desired, cnt
	yaw = msg
	if(cnt==0):
		yaw_desired=yaw
		cnt = 1
	
	# straightLine_pid_imu()
	
def callback_gui(config, level):
    rospy.loginfo("""Reconfigure Request: {KP_yaw}, {KI_yaw}, {KD_yaw}""".format(**config))
    global KP_y, KI_y, KD_y, err_y_thresh
    KP_y, KI_y, KD_y = config.KP_yaw / startpwm, config.KI_yaw / 100, config.KD_yaw / 100 
    err_y_thresh = config.err_y_thresh
    return config

def pid_straight_cb(msg):
	global KP_y, KD_y, KI_y, KP_a, KD_a, KI_a

	KP_y, KD_y, KI_y = msg.KP_yaw, msg.KD_yaw, msg.KI_yaw
	KP_a, KD_a, KI_a = msg.KP_a, msg.KD_a, msg.KI_a

	print(KP_y, KD_y, KI_y)

def dist_r_cb(msg):

	global desired_dist_rt, dist_rt

	if(desired_dist_rt == -17 and con_rt > 75):
		desired_dist_rt = msg.data

	dist_rt = msg.data

def con_r_cb(msg):

	global con_rt
	con_rt = msg.data

if __name__ == "__main__":
	
	
	rospy.init_node("only_straight_pid", anonymous = False)
	q = 10000

	pub=rospy.Publisher("PWM_VALUE_car",String ,queue_size=q)

	# startTime = time.time()
	# while(time.time() < startTime + 7):
	# 	stopCar()
	# pwm_fr, pwm_fl, pwm_br, pwm_bl = startpwm, startpwm, startpwm, startpwm

	rospy.Subscriber("angle_z", Float64, yaw_callback, queue_size=1000)
	rospy.Subscriber("accx", Float64, accx_callback)
	rospy.Subscriber("accy", Float64, accy_callback)
	# rospy.Subscriber("pid_straight", pid_straight_msg, pid_straight_cb)
	rospy.Subscriber("dist_r", Float64, dist_r_cb)
	rospy.Subscriber("con_r", Float64, con_r_cb)

	# srv = Server(PID_yawConfig, callback_gui)

	# signal.signal(signal.SIGINT, sigint_handler)

	rate = rospy.Rate(1)

	# while (1):
	# 	straightLine_pid_imu()
	# 	rate.sleep()

	rospy.spin()


