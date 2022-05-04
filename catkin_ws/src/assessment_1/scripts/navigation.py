# Lewis Robinson - ROB15611294
# CMP3103M - Autonomous Mobile Robotics Assessment 1
# Code adapted from https://app.theconstructsim.com/#/Course/58
# and
# https://github.com/LCAS/teaching
#!/usr/bin/env python

# Import python libraries
import numpy as np
import cv2
import time as tm

from robot_control import RobotControl

class Navigation:

    def __init__(self, speed, time):
        self.rc = RobotControl()
        self.motion = None
        self.speed = speed
        self.time = time
        self.d = None
        self.time_taken = tm.time()
        self.starting = True

    def robot_move(self):
        # Initially get the distance
        distance = self.rc.get_laser(320)
        # Repeat until interrupted (Ctrl+C)
        while True:
            # Move if the robot is further than a specified unit away from any walls
            while (all(d > 1 for d in distance)):
                # Check if last distance read has changed
                # If not, the robot is stuck and needs to reverse
                if (self.stuck(distance, self.time_taken, self.starting)):
                    print("reversing")
                    #self.rc.reverse(self.motion, self.speed, self.time)
                else:
                    self.starting = False
                    # Check for either red/blue/green
                    colour, mask, avg = self.colour_detect(self.rc.img_msg)
                    print(avg)
                    # If the detected colour is above the threshold, then take action
                    # If any red on screen, move away from it
                    if (colour == "red") and (avg > 20):
                        print("moving away")
                        self.rc.move_away(mask)
                        distance = self.rc.get_laser(320)

                    # If any blue or green on screen, move towards it
                    elif (colour == "blue" and avg > 20) or (colour == "green" and avg > 2):
                        print("moving towards")
                        self.rc.move_towards(mask)
                        distance = self.rc.get_laser(320)
                        
                    # Else just move normally
                    else:
                        print("moving normally")
                        # Move the robot forward and scan again
                        self.rc.move_straight()
                        distance = self.rc.get_laser(320)
                        #print("Distance from wall : ", distance)

            # If the robot is too close to the wall, stop it and initiate turning
            self.rc.stop_robot()
            self.motion = self.robot_turn() # Calculate the amount to turn
            self.rc.turn(self.motion, self.speed, self.time) # Turn the robot by the amount calculated
            distance = self.rc.get_laser(320)
            self.time_taken = tm.time() # Record time at turn
            self.starting = False # Reset after turning, so the robot doesn't instantly reverse

    def robot_turn(self):
        # Get all of the laser scan distances
        self.d = self.rc.get_laser_full()

        # Split them into the right and left side (laser scan numbers are reversed because it goes clockwise)
        count = len(self.d)
        right_dist = self.d[:count/2]
        left_dist = self.d[-(count/2):]

        # Calculate the averages of the right and left scans
        right_avg = np.nansum(right_dist) / len(right_dist)
        left_avg = np.nansum(left_dist) / len(left_dist)

        # If the left wall is closer, rotate clockwise away from it
        if(right_avg > left_avg):
            self.d = None
            #print("Turning Clockwise")
            return "clockwise"
        # If the right wall is closer, rotate anti-clockwise away from it
        else:
            self.d = None
            #print("Turning Anti-Clockwise")
            return "anti-clockwise"

    def colour_detect(self, cv_image):
        self.starting = False
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create separate masks for red/blue/green
        hsv_mask_red = cv2.inRange(hsv_img, np.array((0,150,20)), np.array((20,255,255)))
        hsv_mask_blue = cv2.inRange(hsv_img, np.array((110,50,50)), np.array((130,255,255)))
        hsv_mask_green = cv2.inRange(hsv_img, np.array((50,50,50)), np.array((70,255,255)))
        
        # Create red/blue/green moments to detect if a colour is on screen
        M_R = cv2.moments(hsv_mask_red)
        M_B = cv2.moments(hsv_mask_blue)
        M_G = cv2.moments(hsv_mask_green)
        
        # Return the colour detected, relevant mask, and the size of the object
        if M_R['m00'] > 0:
            return "red", M_R, cv2.mean(hsv_mask_red)[0]
        elif M_B['m00'] > 0:
            return "blue", M_B, cv2.mean(hsv_mask_blue)[0]
        elif M_G['m00'] > 0:
            return "green", M_G, cv2.mean(hsv_mask_green)[0]
        else:
            return "none", 0, 0

    def stuck(self, distance, time_taken, starting):
        # Calculate distance travelled from previous laser scan
        dist_t = distance[len(distance)/2] - self.rc.get_front_laser()
        print(dist_t)
        # Calculate time since last turn, so robot doesn't reverse after every turn
        prev_time = tm.time() - time_taken
        #print(prev_time)
        # If little/no distance travelled, robot is stuck
        if (-0.001 < dist_t < 0.001) and (prev_time < 2.5) and (starting == False):
            return True
        else:
            return False

robot = Navigation(speed=1.0, time=5)
robot.robot_move()