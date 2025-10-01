1. Install the drivers 

```bash
   sudo apt-get install ros-humble-ur
```
   
2. Robot IP: `192.168.0.102`

3. Run calibration
```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.0.102 target_filename:="${HOME}/my_robot_calibration.yaml"
```

4. Run driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.102 launch_rviz:=false headless_mode:=true
```

5. Run moveit
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

## Simulation Setup

1. Run Simulation
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.102 launch_rviz:=false headless_mode:=true
```

2. Run moveIt
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_sim_time:=true
```

## MoveIt Solution
1. Run Camera
```bash
ros2 launch depthai_ros_driver camera.launch.py parent_frame:=world cam_pos_x:=0.862 cam_pos_y:=0.17  cam_pos_z:=1.175 cam_yaw:=3.095 cam_pitch:=0.455 cam_roll:=-0.005
```

2. Run simulation
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5 launch_rviz:=false
```

3. Run moveit
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_sim_time:=true
```

4. Bringup solution
```bash
ros2 launch follow_traj_moveit system_bringup.launch.py use_sim_time:=true
```
