1. Run Vicon calibration. Move Panda to center of room. Add markers at flange. Name the object. 
2. Clone and establish vicon_bridge on the Panda computer. Ensure that the ip address is set correctly.
```
source /opt/ros/noetic/setup.bash
roslaunch vicon_bridge vicon.launch
```
2. Ensure the bridge is running correctly. Check rviz. Check tf to see world frame name.
```
rostopic list | grep vicon
```
3. Test moveit
  ```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch \
  load_gripper:=false robot_ip:=franka2
```
Open a second terminal and run:
```
python3 automate_moveit_calibration_ros1.py --num-poses 2
```
Then execute:
```
python3 automate_moveit_calibration_ros1.py \
  --num-poses 2 \
  --execute
```
5.  Capture vicon topics with the automated poses. Double check vicon world frame name.
```
python3 automate_moveit_calibration_ros1.py \
  --num-poses 2 \
  --execute \
  --capture \
  --vicon-world vicon_world \
  --marker-frame panda_calib \
  --out calibration_pairs.json
```
Change number of poses for full calibration. 
6. Run the solver:
```
python3 solve_calibration.py calibration_pairs.json \
  --out calibration_result.json
```
7. Calibrate probe. 
