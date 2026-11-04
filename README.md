1. Turn on Vicon to run for more than 30 minutes.
2. Turn on the Panda. Weigh the peg and replace the end effector. Make sure the robot is locked out. (can maybe try uploading the stl of the peg?) 
3. Launch the controller. Try to change the stiffness? Try to change the desired goal pose? Potentially install plotjuggler?
   - Create a node or edit launch to publish the desired torque of the controller. Add this topic to the list of rosbag record topics.
4. Calibrate Vicon.
5. Move Panda to center of room. Add markers on the Panda. Create object in Vicon - orient axis towards the other point. take a picture of setup.
6. Add markers on the probe (need to pick up attachment). Create object in Vicon - orient axis towards other point. take picture of setup.
7. Launch Vicon bridge
```
source /opt/ros/noetic/setup.bash
roslaunch vicon_bridge vicon.launch
```
7. Ensure the bridge is running correctly. Check rviz. Check tf to see world frame name. Add marker to rviz (set frame to vicon_world). Make sure the markers have timestamped data. 
```
rostopic list
rviz
```

8. Collect data. Check procedure in document. 
```
rosbag record -O point1_trial1 \
  /joint_states \
  /tf \
  /tf_static \
  /franka_state_controller/franka_states \
  /franka_state_controller/joint_states_desired \
  /franka_state_controller/F_ext \
ADD THE DESIRED TORQUE TOPIC
ADD VICON MARKER TOPICS
```
9. Check test trial 
rosbag info test_recording.bag

*** Ensure to divide force properly to obtain N post-processing***
