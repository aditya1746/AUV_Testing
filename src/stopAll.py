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

pwm_mr, pwm_ml = 1470 , 1470
pwm_fr, pwm_fl, pwm_br, pwm_bl = 1470,1470,1470,1470

def stopMiddle():
    pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
    pub_middle.publish(pwm_msg)

def startMiddle():
    pwm_mr, pwm_ml = 100, 100
    pwm_mr, pwm_ml = 1470 - int(pwm_mr), 1470 - 1*int(pwm_ml)
    pwm_msg = str(pwm_mr) + ' ' + str(pwm_ml) + ' '
    pub_middle.publish(pwm_msg)
	
def stopCar():
    pwm_msg = str(pwm_fr) + ' ' + str(pwm_fl) + ' ' + str(pwm_br) + ' ' + str(pwm_bl) + ' '
    pub_car.publish(pwm_msg)

def depth_callback(msg):
    
    stopCar()
    stopMiddle()
    
if __name__ == "__main__":

    rospy.init_node("stopall_node", anonymous = False)
    q = 1
        
    rospy.Subscriber("Depth", Float32, depth_callback)

    pub_middle=rospy.Publisher("PWM_VALUE_middle",String ,queue_size=q)
    pub_car=rospy.Publisher("PWM_VALUE_car",String ,queue_size=q)
    # srv = Server(PID_yawConfig, callback_gui)
    
    rospy.spin()
