#! /usr/bin/env python
from numpy import float32
import rospy
from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
# from auv_codes2.cfg import PID_yawConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
# from auv_codes2.msg import pid_depth_msg

#    FL BL FR BR ML MR
# F  C  C  C  C 
# B  A  A  A  A
# L  A  C  C  A
# R  C  A  A  C
# D  -  -  -  -  C  C
# U  -  -  -  -  A  A

prev_err_d, int_err_d = 0,0         # d -> depth
err_d_thresh = 0.000                # meters
prev_err_r, int_err_r = 0,0         # r -> roll
err_r_thresh = 1                  # degrees

depth = 0
desiredDepth = 0.7      # meters
roll = 0
desiredRoll = 0
cnt = 0
initialDepth = 1000

# KP_d, KD_d, KI_d = 250.0,17.0,0.0
KP_d, KD_d, KI_d = 2000.0,400.0,0.0
# KP_d, KD_d, KI_d = 0.0,0.0,0.0
KP_r, KD_r, KI_r = 5.0 ,4.0,0.0

startPwm = 0
MAXpwm, MINpwm = 200, -200
pwmBase_r, pwmBase_l = 1480, 1480
pwm_mr, pwm_ml = 0 , 0

correction_d, correction_r  = 0.0, 0.0

# x-axis ^ 
# y-axis >

def depth_pid_pressure():
    global prev_err_d, int_err_d, prev_err_r, int_err_r
    global pwm_mr, pwm_ml, correction_d, correction_r
	# pwm_mr, pwm_ml = 100, 100  # ================== 100 basically represent base velocities
    
    err_d = desiredDepth - depth # +err => or niche jana he
    diff_err_d = err_d - prev_err_d
    int_err_d += err_d 
    prev_err_d = err_d

    print('error d= ',err_d)
    correction_d = KP_d*err_d + KD_d*diff_err_d + KI_d*int_err_d
    
    
    err_r = desiredRoll - roll # +err => right side niche jhuk gai he
    diff_err_r = err_r - prev_err_r
    int_err_r += err_r 
    prev_err_r = err_r
    print('error r= ',err_r)

    pwm_mr= 100
    pwm_ml= 100


    # print('error r= ',err_r)
    correction_r = KP_r*err_r + KD_r*diff_err_r + KI_r*int_err_r

    pwm_mr += correction_d
    pwm_ml += correction_d

    pwm_mr -= correction_r
    pwm_ml += correction_r

    pwm_mr = max(pwm_mr, MINpwm) 
    pwm_mr = min(pwm_mr, MAXpwm) 
    pwm_ml = max(pwm_ml, MINpwm) 
    pwm_ml = min(pwm_ml, MAXpwm) 

    print(pwm_mr, pwm_ml)
    
    mr, ml = pwmBase_r - int(pwm_mr), pwmBase_l - 1*int(pwm_ml)
    
    pwm_msg = str(mr) + ' ' + str(ml) + ' '
    pub.publish(pwm_msg)
    # rospy.loginfo(depth)

def stopMiddle():
    pwm_mr, pwm_ml = pwmBase_r, pwmBase_l
    pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
    pub.publish(pwm_msg)

def startMiddle():
    pwm_mr, pwm_ml = 100, 100
    pwm_mr, pwm_ml = pwmBase_r - int(pwm_mr), pwmBase_l - int(pwm_ml)
    pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
    pub.publish(pwm_msg)
	

def callback_gui(config, level):
    rospy.loginfo("""Reconfigure Request: {KP_depth}, {KI_depth}, {KD_depth}""".format(**config))
    global KP_d, KI_d, KD_d
    KP_d, KI_d, KD_d = config.KP_depth / 100, config.KI_depth / 100, config.KD_depth / 100 
    return config

def depth_callback(msg):
    global depth, initialDepth
    depth = msg.data
    # print(depth)
    if(initialDepth==1000):
        initialDepth = depth

    depth = depth - initialDepth
    # print(depth)
    depth_pid_pressure()
    # startMiddle()

def depthCBinGUI(msg):
    global depth
    depth = msg
    depth_pid_pressure()

def roll_callback(msg):
    global roll, cnt, desiredRoll
    roll = msg.data
    if(cnt==0):
        desiredRoll = roll
        cnt = 1

    # print(roll)

def pid_depth_cb(msg):
    global KP_d, KD_d, KI_d, KP_r, KD_r, KI_r
    
    KP_d, KD_d, KI_d = msg.KP_depth, msg.KD_depth, msg.KI_depth
    KP_r, KD_r, KI_r = msg.KP_r, msg.KD_r, msg.KI_r
    
    print(KP_d, KD_d, KI_d)
    
if __name__ == "__main__":

    rospy.init_node("only_depth_pid", anonymous = False)
    q = 1
        
    rospy.Subscriber("Depth", Float32, depth_callback)
    rospy.Subscriber("angle_y", Float64, roll_callback)

    pub=rospy.Publisher("PWM_VALUE_middle",String ,queue_size=q)
    # rospy.Subscriber("pid_depth", pid_depth_msg, pid_depth_cb, queue_size=1000)
    # srv = Server(PID_yawConfig, callback_gui)
    
    rospy.spin()
