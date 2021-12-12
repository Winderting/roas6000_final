# roas6000_final
cd to catkin_workspace/src, git clone this repo, donot rename it (or if you rename it, the following command should be modified correspondingly)
cd to catkin_workspace, catkin_make

for vrep simulation
terminal 1 - start the ros master
$ roscore
terminal 2 - open the vrep, load the env.ttt, start the simulation
terminal 3 - start slam, rviz showing the map gotten from slam, and teleop_control
$ roslaunch roas6000_final roas_teleop.launch 
terminal 4 - open the face-recognition, and publish the marker if detected face
$ cd catkin_workspace/src/roas6000_final/
$ rosrun roas6000_final dlib_detect.py
if not excutable, run the following command
$ chmod +x ./scripts/dlib_detect.py
"theta_x" error is because when face is not detected at first, there will be not relative postion of face to image center, so not theta_x, just ignore it and this will be corrected later.

for rosbag recorded file (no rosbag updated for now)
rosparam set use_sim_time True
