# CMP3103M-Autonomous-Mobile-Robotics-Assessment1
<p>Reactive robot behaviour, programmed in Python using ROS that enables a robot to find a goal in a maze simulation.</p>
The robot, simulated in Gazebo, uses 3 ROS topics to navigate the maze. It subscribes to the LaserScan and Image data and publishes to the Velocity node. The navigation algorithm works by determining the average distances of both sides of the robot, then rotating towards the direction where there is more space. Also if the camera detects colour, it will act depending on it, moving towards blue/green and away from red.

<ol>
  <p>
    <li>Run this command to make sure software is up to date:<br>
    <code>sudo apt update && sudo apt upgrade && sudo apt install ros-melodic-uol-cmp3103m</code></li>
  </p>
  <p>
    <li>Launch the any of the 3 maze environments:<br>
    <code>roslaunch uol_turtlebot_simulator maze1.launch</code><br>
    or<br>
    <code>roslaunch uol_turtlebot_simulator maze2.launch</code><br>
    or<br>
    <code>roslaunch uol_turtlebot_simulator maze3.launch</code><br></li>
  </p>

  <li>Launch the sensor visualisation(optional):<br>
  <code>roslaunch uol_turtlebot_simulator turtlebot-rviz.launch</code></li>
  
  <li>Run the navigation code. Path:<br>
  <code>catkin_ws/src/assessment_1/scripts/navigation.py</code></li>

</ol>

# Video Demonstration
[![Demonstration](http://img.youtube.com/vi/8BwjYr6XPkk/0.jpg)](http://www.youtube.com/watch?v=8BwjYr6XPkk "CMP3103M-AMR-Assessment1")
