import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

__author__ = "Syed Shanto" 
__email__ = "sy670205@dal.ca" 
__date__ = "5/26/21"


class TriangleMove(object):
   
    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "triangle_move"
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
        while (((time.time() - self.t_init) < 1) and (not ros.is_shutdown())):
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


class TriangleMoveVel(TriangleMove):
    """
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt2.py vel
    """

    def __init__(self):
        super(TriangleMoveVel, self).__init__()

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
        # duration, speed format so for next line 2 is duration and 0.5 is speed
	# all roation until 180 degree is counter clockwise
	# forward speed = 0.15 m/s and angspeed = 1 rad/s
	self.go_forward(4.3, 0.15) # set the value in such a way that it goes 0.5 m forward
        self.turn(3.5, 1) # set the value for 45 degree rotation  
        # self.go_forward(2, 0.5) # set the value for 0.71m forward
        # self.turn(3.5, 0.5) # set the value for 45 degree rotation
        # self.go_forward(2, 0.5) # set the value for 0.5m forward
	# completes triangle counter clockwise#
	#self.turn(3.5, 0.5) # turn 180 degree
	# All rotation now should be clockwise for retracing
	#self.go_forward(2, 0.5) # set the value for 0.5m forward
	#self.turn(3.5, 0.5) # turn 45 degree
	#self.go_forward(2, 0.5) # set the value for 0.71m forward
	#self.turn(3.5, 0.5) # turn 45 degrees
	#self.go_forward(2, 0.5) # set the value for 0.5m forward  
        # robot should be at origin now
	self.stop_robot() # move process stopped
	
	
#class TriangleMoveOdom(TriangleMove):

	# we are not implementing odom 
	# as it requiremore math
	# maybe when i have more time later I will check it out.
	

if __name__ == '__main__':

    if len(sys.argv) > 1:

        if sys.argv[1] == "odomvel":
            r = TriangleMoveVel()

        else:
	    print("Wrong arguments have been passed in");
            sys.exit(-1)
    else:
        sys.exit(-1)
    r.start_ros()
    r.move()

