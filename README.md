# ROAS6000_final  
## Before Start
require installed ros melodic and vrep(coppeliaSim), and sim_ros_interface (https://github.com/CoppeliaRobotics/simExtROS) for communication between them.  
require some packages including hector-slam, cv2, dlib, face-recognition and so on 

cd to catkin_workspace/src, git clone this repo, donot rename it (or if you rename it, the following command should be modified correspondingly)  
cd to catkin_workspace, catkin_make  
  
run the following command for the .py in catkin_workspace/src/roas6000_final/script, for example:(otherwise when running the following terminal 3, there may be no response)     
$ chmod +x catkin_workspace/src/roas6000_final/scripts/dlib_detect.py  
  
## For VREP Simulation:  
terminal 1 - start the ros master  
  
$ roscore  
terminal 2 - open the vrep, load the env.ttt, start the simulation  
  
terminal 3 - start slam, face detector, rviz showing the map gotten from slam and the marker representing the location of the face image if face detected, teleop_control to control the robot, area_check to check which area of room the robot is in.  
$ roslaunch roas6000_final roas_main.launch   
  
terminal 4 - start the visual servoing when reach the D area  
$ rosrun roas6000_final visual_servoing.py  
  
## For rosbag recorded file, 
first you need to set the ros to use simulation time  
rosparam set use_sim_time True  
