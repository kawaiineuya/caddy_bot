#!/usr/bin/env python2
import rospy
import tf.transformations
import Jetson.GPIO as GPIO
from geometry_msgs.msg import Twist

WHEEL_RADIUS = 0.132 # meter
WHEEL_SEPARATION = 0.566 # meter
TURNING_RADIUS = 0.283 #meter
ROBOT_RADIUS = 0.600 #meter

MOTOR_MAX = 30 #-100 ~ 100
MOTOT_MIN = -MOTOR_MAX

MAX_LINEAR_VELOCITY = (WHEEL_RADIUS * 2 * 3.14159265359 * 172 / 60) # m/s 2.38889 rpm 172??
MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / TURNING_RADIUS) # rad/s

MIN_LINEAR_VELOCITY = -MAX_LINEAR_VELOCITY  
MIN_ANGULAR_VELOCITY = -MAX_ANGULAR_VELOCITY 

CONTROL_MOTOR_SPEED_FREQUENCY =  30 # hz

LINEAR = 0
ANGULAR = 1

GOAL_VELOCITY_FROM_CMD = [0.0, 0.0]

LEFT = 0
RIGHT = 1

WHEEL_VELOCITY_CMD = [0.0, 0.0]

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

DIR1 = 37
DIR2 = 18
PWM1 = 32
PWM2 = 33

Frequency = 1000
DutyCycle = 30
Stop = 0

GPIO.setup(PWM2, GPIO.OUT)
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(DIR1, GPIO.OUT)

p1 = GPIO.PWM(PWM1, Frequency)
p2 = GPIO.PWM(PWM2, Frequency)
p1.start(Stop)
p2.start(Stop)

def remap( x, oMin, oMax, nMin, nMax ):

    #range check
    if oMin == oMax:
        print "Warning: Zero input range"
        return None

    if nMin == nMax:
        print "Warning: Zero output range"
        return None

    #check reversed input range
    reverseInput = False
    oldMin = min( oMin, oMax )
    oldMax = max( oMin, oMax )
    if not oldMin == oMin:
        reverseInput = True

    #check reversed output range
    reverseOutput = False
    newMin = min( nMin, nMax )
    newMax = max( nMin, nMax )
    if not newMin == nMin :
        reverseOutput = True

    portion = (x-oldMin)*(newMax-newMin)/(oldMax-oldMin)
    if reverseInput:
        portion = (oldMax-x)*(newMax-newMin)/(oldMax-oldMin)

    result = portion + newMin
    if reverseOutput:
        result = newMax - portion

    return result

# def remap(x, in_min, in_max, out_min, out_max):
#     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def motor(motor):
    left = motor[0]
    right = motor[1]
    
    if left < 0 :
        GPIO.output(DIR1, GPIO.LOW)
        p1.start(-left)
    elif left == 0:
        GPIO.output(DIR1, GPIO.LOW)
        p1.start(Stop)
    elif left > 0 :
        GPIO.output(DIR1, GPIO.HIGH)
        p1.start(left)
    
    if right < 0 :
        GPIO.output(DIR2, GPIO.LOW)
        p2.start(-right)
    elif right == 0:
        GPIO.output(DIR2, GPIO.LOW)
        p2.start(Stop)
    elif right > 0 :
        GPIO.output(DIR2, GPIO.HIGH)
        p2.start(right)

def StopMotors():
   GPIO.output(DIR1, GPIO.LOW)
   GPIO.output(DIR2, GPIO.LOW)
   p1.start(Stop)
   p2.start(Stop)

def callback(msg):
    # rospy.loginfo("Received a /cmd_vel message!")
    # rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    # 2022 08 11 
    # GOAL_VELOCITY_FROM_CMD[LINEAR] = constrain(msg.linear.x,  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
    # GOAL_VELOCITY_FROM_CMD[ANGULAR] = constrain(msg.angular.z,  MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
    GOAL_VELOCITY_FROM_CMD[LINEAR] = remap(msg.linear.x,-1,1, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
    GOAL_VELOCITY_FROM_CMD[ANGULAR] = remap(msg.angular.z ,-1, 1, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)

    # rospy.loginfo(GOAL_VELOCITY_FROM_CMD)

    WHEEL_VELOCITY_CMD[LEFT] = GOAL_VELOCITY_FROM_CMD[LINEAR] - (GOAL_VELOCITY_FROM_CMD[ANGULAR] * WHEEL_SEPARATION / 2)
    WHEEL_VELOCITY_CMD[RIGHT] = GOAL_VELOCITY_FROM_CMD[LINEAR] + (GOAL_VELOCITY_FROM_CMD[ANGULAR] * WHEEL_SEPARATION / 2)

    # rospy.loginfo(WHEEL_VELOCITY_CMD)

    WHEEL_VELOCITY_CMD[LEFT] = constrain(WHEEL_VELOCITY_CMD[LEFT],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
    WHEEL_VELOCITY_CMD[RIGHT] = constrain(WHEEL_VELOCITY_CMD[RIGHT],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)

    # rospy.loginfo(WHEEL_VELOCITY_CMD)

    WHEEL_VELOCITY_CMD[LEFT] = remap(WHEEL_VELOCITY_CMD[LEFT],MIN_LINEAR_VELOCITY,MAX_LINEAR_VELOCITY,-100,100)
    WHEEL_VELOCITY_CMD[RIGHT] = remap(WHEEL_VELOCITY_CMD[RIGHT],MIN_LINEAR_VELOCITY,MAX_LINEAR_VELOCITY,-100,100)

    if(WHEEL_VELOCITY_CMD[LEFT] > 20):
        WHEEL_VELOCITY_CMD[LEFT] = 20
    elif(WHEEL_VELOCITY_CMD[LEFT] < -20):
        WHEEL_VELOCITY_CMD[LEFT] = -20
    if(WHEEL_VELOCITY_CMD[RIGHT] > 20):
        WHEEL_VELOCITY_CMD[RIGHT] = 20
    elif(WHEEL_VELOCITY_CMD[RIGHT] < -20):
        WHEEL_VELOCITY_CMD[RIGHT] = -20

    # rospy.loginfo("Linear, Angular: %f, %f \tMotor left, rigth: %f, %f"%(GOAL_VELOCITY_FROM_CMD[LINEAR],GOAL_VELOCITY_FROM_CMD[ANGULAR],WHEEL_VELOCITY_CMD[LEFT],WHEEL_VELOCITY_CMD[RIGHT]))
    if GOAL_VELOCITY_FROM_CMD[LINEAR] > 0:
        if GOAL_VELOCITY_FROM_CMD[ANGULAR] > 0:
            rospy.loginfo("go, left")
        elif GOAL_VELOCITY_FROM_CMD[ANGULAR] < 0:
            rospy.loginfo("go, right")
        elif GOAL_VELOCITY_FROM_CMD[ANGULAR] == 0:
            rospy.loginfo("go")    
    elif GOAL_VELOCITY_FROM_CMD[LINEAR] < 0:
        if GOAL_VELOCITY_FROM_CMD[ANGULAR] > 0:
            rospy.loginfo("back, left")
        elif GOAL_VELOCITY_FROM_CMD[ANGULAR] < 0:
            rospy.loginfo("back, right")
        elif GOAL_VELOCITY_FROM_CMD[ANGULAR] == 0:
            rospy.loginfo("back")
    elif GOAL_VELOCITY_FROM_CMD[LINEAR] == 0:
        if GOAL_VELOCITY_FROM_CMD[ANGULAR] > 0:
            rospy.loginfo("left")
        elif GOAL_VELOCITY_FROM_CMD[ANGULAR] < 0:
            rospy.loginfo("right")
        elif GOAL_VELOCITY_FROM_CMD[ANGULAR] == 0:
            rospy.loginfo("stop")  

    motor(WHEEL_VELOCITY_CMD)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    StopMotors()
    GPIO.cleanup()