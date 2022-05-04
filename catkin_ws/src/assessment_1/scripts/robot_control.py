# Lewis Robinson - ROB15611294
# CMP3103M - Autonomous Mobile Robotics Assessment 1
# Code adapted from https://app.theconstructsim.com/#/Course/58
# and
# https://github.com/LCAS/teaching
#!/usr/bin/env python

# Import python libraries
import rospy
import numpy as np
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import ROS topics
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.bridge = CvBridge()
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.err_pub = rospy.Publisher('error', Float64, queue_size=1)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.img_msg = Image()
        self.ctrl_c = False
        self.rate = rospy.Rate(5)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_pub.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(self.cmd)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        time.sleep(0.5)
        # Get range of lasers, not just the front
        laser_range = list(self.laser_msg.ranges[pos-60:pos+60])
        # Remove null values
        newlist = [x for x in laser_range if np.isnan(x) == False]
        return newlist

    def get_front_laser(self):
        time.sleep(0.5)
        return self.laser_msg.ranges[320]

    def get_laser_full(self):
        time.sleep(0.5)
        return self.laser_msg.ranges

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.3
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_pub.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s

    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_pub.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

    def reverse(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = -0.2
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_pub.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()
    def image_callback(self, data):
        try:
            self.img_msg = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def move_towards(self, M):
        h, w, d = self.img_msg.shape

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print('cx: %f, cy: %f' %(cx, cy))
        cv2.circle(self.img_msg, (cx, cy), 20, (255, 0, 0), -1)
        err = cx - w/2
        self.cmd.linear.x = 0.3
        self.cmd.angular.z = -float(err) / 200

        self.vel_pub.publish(self.cmd)
        self.rate.sleep()
        self.err_pub.publish(err)

        cv2.imshow("Image window", self.img_msg)
        cv2.waitKey(1)

    def move_away(self, M):
        h, w, d = self.img_msg.shape

        cx = int(M['m10']/M['m00'])
        err = cx - w
        #self.cmd.linear.x = -0.5
        self.cmd.angular.z = 10

        self.vel_pub.publish(self.cmd)
        self.err_pub.publish(err)

if __name__ == '__main__':
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()
    except rospy.ROSInterruptException:
        pass
