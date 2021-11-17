#!/usr/bin/env python3

from time import time
import rospy
from geometry_msgs.msg import Twist
from rospy.core import loginfo
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# create velocity_msg Twist object to publish messages to /cmd_vel topic
velocity_msg = Twist()

# declare a global publisher variable
pub=None

# declare a function for movement of ebot
def move(linear,angular):
    global pub,velocity_msg
    velocity_msg.linear.x = linear
    velocity_msg.angular.z = angular
    pub.publish(velocity_msg)

# declare global regions for laser callback
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}

# create a global state variable to store the current state of the ebot
state_ = 0

# declare different state of ebot
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

# declare a function for rotation of ebot
def rotate(linear,angular):
    global pub,velocity_msg
    velocity_msg.linear.x = linear
    velocity_msg.angular.z = -angular
    pub.publish(velocity_msg)


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def laser_callback(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    take_action()


# declare a function for wall following
def take_action():
    global regions_,velocity_msg
    regions = regions_
    velocity_msg.linear.x = 0
    velocity_msg.angular.z=0
    
    state_description = ''
    
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)


# declare a function for obstacle avoiding
def avoid_wall():
    global velocity_msg,regions_
    regions=regions_
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 1 - nothing'
        linear_x = 0.7
        angular_z = 0
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < .8 and regions['fleft'] < .8 and regions['fright'] < .8:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.7
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    velocity_msg.linear.x = linear_x
    velocity_msg.angular.z = angular_z
    pub.publish(velocity_msg)

# declare a function for changing the state of ebot
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        # print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        print("wall follower {}".format(state, state_dict_[state]))
        state_ = state
        
        
def find_wall():
    global velocity_msg
    velocity_msg.linear.x = 0.2
    velocity_msg.angular.z = -0.3
    return velocity_msg

def turn_left():
    velocity_msg
    velocity_msg.angular.z = 0.3
    return velocity_msg

def follow_the_wall():
    global regions_,velocity_msg
    velocity_msg.linear.x = 0.7
    return velocity_msg


def control_loop():
    global velocity_msg,pub,state_
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(20)
    rotaterate=rospy.Rate(100) 
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    t0 = rospy.Time.now().to_sec()
    sum=0
    travel_d=0

    #functions 1 2 3 4 to rotate and move ebot on the desired path
    while not rospy.is_shutdown():        #function 1
                angle=math.radians(60)
                t3 = rospy.Time.now().to_sec()
                rotaterate.sleep()
                move(0,angle)
                t4 =  rospy.Time.now().to_sec()
                rotaterate.sleep() 
                sum+=(t4-t3)
                rospy.loginfo(sum)
                if(sum >= 1.4) :
                    move(0,0)
                    sum=0
                    rospy.loginfo(sum)
                    break
    
    while not rospy.is_shutdown():       #function 2
        move(0.7,0)
        t1 =  rospy.Time.now().to_sec()
        rate.sleep()
        travel_d = (t1-t0) * 0.7 
        rospy.loginfo(travel_d)
        if (travel_d>3.5): 
            move(0,0)
            rospy.loginfo('Flag 3')
            travel_d= 0
            break

    while not rospy.is_shutdown():       #function 3
                angle=math.radians(60)
                t3 = rospy.Time.now().to_sec()
                rotaterate.sleep()
                rotate(0,angle)
                t4 =  rospy.Time.now().to_sec()
                rotaterate.sleep() 
                sum+=(t4-t3)
                rospy.loginfo(sum)
                if(sum >= .8) :
                    move(0,0)
                    sum=0
                    rospy.loginfo(sum)
                    break
                
    # calling function for to follow wall            
    while not rospy.is_shutdown():
        if state_ == 0:
            velocity_msg = find_wall()
        elif state_ == 1:
            velocity_msg = turn_left()
        elif state_ == 2:
            velocity_msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        pub.publish(velocity_msg)
        t1=rospy.get_time()
        sum+=t1
        rospy.loginfo(sum)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()
        if(sum>=54000):
            move(0,0)
            sum=0
            break
        
        
    while not rospy.is_shutdown():      # function 4
                angle=math.radians(60)
                t3 = rospy.Time.now().to_sec()
                rotaterate.sleep()
                move(0,-angle)
                t4 =  rospy.Time.now().to_sec()
                rotaterate.sleep() 
                sum+=(t4-t3)
                if(sum >= 2.0) :
                    move(0,0)
                    sum=0
                    rospy.loginfo("flag 2")
                    break
    
    # calling function to avoid wall
    while not rospy.is_shutdown(): 
        move(1,0)
        t1=rospy.get_time()
        avoid_wall()
        rate.sleep()
        sum+=t1
        rospy.loginfo(sum)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()
        if(sum>=14500):
            move(0,0)
            sum=0
            break



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass