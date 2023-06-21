#!/usr/bin/python
import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SquareMove(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "square_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        self.odometry_sub = None

        # ROS params
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def move(self):
        """ To be surcharged in the inheriting class"""

        while not ros.is_shutdown():
            time.sleep(1)

    def __odom_ros_sub(self, msg):

        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)




class SquareMoveVel(SquareMove):
    """
    This class implements a open-loop square trajectory based on velocity control. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py vel
    """

    def __init__(self):
        
        super(SquareMoveVel, self).__init__()

    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def turn(self, duration, ang_speed):

         # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait 2 sec (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = ang_speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def move(self):

        self.go_forward(2, 0.5)         #movement will depend on code in Odom
        self.turn(3.5, 0.5)             #robot turns 90deg
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)            #robot turns 90deg
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)            #robot turns 90deg
        self.go_forward(2, 0.5)
        self.stop_robot()              #robot will stop at the origin


class SquareMoveOdom(SquareMove):
    """
    This class implements a semi closed-loop square trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py odom
    """

    def __init__(self):


        super(SquareMoveOdom, self).__init__()

        self.pub_rate = 0.1

    def get_z_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = (yaw*180)/math.pi
#	print yaw
        return yaw
        
    def move_of(self, d, speed=0.15):

        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not ros.is_shutdown():

           # sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
           # (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
           # sys.stdout.flush()

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def turn_of(self, a, ang_speed=0.2):

        # Convert the orientation quaternion message to Euler angles
        # a_init = self.get_z_rotation(self.odom_pose.orientation)
        # print a_init, a

        # Set the angular velocity forward until angle is reached
	while(not ros.is_shutdown()):
	    if(a-1.5 <= self.get_z_rotation(self.odom_pose.orientation) <= a+1.5):
	 	break
       	    # while (self.get_z_rotation(self.odom_pose.orientation) - a_init) < a and not ros.is_shutdown():
            # print self.get_z_rotation(self.odom_pose.orientation), a_init, a
            # sys.stdout.write("\r [TURN] The robot has turned of {:.2f}".format(self.get_z_rotation(self.odom_pose.orientation) - \
            #     a_init) + "rad over {:.2f}".format(a) + "rad")
            # sys.stdout.flush()
            # print (self.get_z_rotation(self.odom_pose.orientation) - a_init)

            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        # sys.stdout.write("\n")

    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)

        # Implement main instructions
        self.move_of(1.88,0.08)          #Start movement
	self.turn_of(89.5,0.25)         #robot turns 90deg
	self.move_of(1.00,0.08)
	self.turn_of(179.5,0.25)
	self.move_of(1.85,0.08)
	self.turn_of(89.5,0.25)
	self.move_of(2.63,0.08)
	self.turn_of(0.8,0.25)
	self.move_of(3.08,0.08)
	self.turn_of(-89.5,0.25)
	self.move_of(2.36,0.08)
	self.turn_of(89.5,0.25)
	self.move_of(2.36,0.08)
	self.turn_of(179.5,0.25)
	self.move_of(3.08,0.08)
	self.turn_of(-89.55,0.25)
	self.move_of(1.05,0.08)
	self.turn_of(0.5,0.25)
	self.move_of(1.29,0.08)
	self.turn_of(-89.5,0.25)
	self.move_of(0.70,0.08)
	self.turn_of(0.5,0.25)
	self.move_of(0.70,0.08)
	self.turn_of(89.5,0.25)
	self.move_of(0.70,0.08)
	self.turn_of(0.5,0.25)
	self.move_of(0.62,0.08)
	self.turn_of(-89.5,0.25)
	self.move_of(1.86,0.08)
	self.turn_of(0.5,0.25)
	self.move_of(1.28,0.08)
	self.turn_of(89.5,0.25)
	self.move_of(2.45,0.08)
	self.turn_of(-89.5,0.25)
	self.move_of(3.07,0.08)
	self.turn_of(0.5,0.25)

if __name__ == '__main__':

    # Choose the example you need to run in the command line
    if len(sys.argv) > 1:

        if sys.argv[1] == "vel":
            r = SquareMoveVel()

        elif sys.argv[1] == "odom":
            r = SquareMoveOdom()

        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    r.move()
