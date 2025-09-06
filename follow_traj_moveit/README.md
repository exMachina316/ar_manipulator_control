# Setuop Steps

1. Install the drivers 

```bash
   sudo apt-get install ros-humble-ur
```
   
2. Robot IP: `192.168.0.103`

3. Run calibration
```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.0.103 target_filename:="${HOME}/my_robot_calibration.yaml"
```

4. Run driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.103 launch_rviz:=false
```

5. Run moveit
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

## Simulation Setup

1. Run Simulation
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5 launch_rviz:=false
```

2. Run moveIt
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_sim_time:=true
```

## MoveIt Solution
1. Run simulation
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5 launch_rviz:=false
```

2. Run moveit
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_sim_time:=true
```

3. Bringup solution
```bash
ros2 launch follow_traj_moveit system_bringup.launch.py use_sim_time:=true
```
