UR Robot IP: `192.168.0.102`

OnRobot Compute box IP: `192.168.1.1`

## Docker Config steps

1. Go to scripts dir

```bash

cd scripts

```

2. Check if image is available

```bash

docker images | grep ur-ft-dev-img:latest

```

if image is not available or you wish to rebuild it

```bash

./build_image.sh

```

3. If container is not available or you wish to create a new container

```bash

./create_container.sh

```

4. If container is available but stopped or exited, bootup the container

```bash

./bootup_container.sh

```

5. To get a bash shell into the docker container

```bash

./enter_bash.sh

```

## Steps to run the hardware

1. Run calibration (Optional-for first time only)

```bash

ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.0.102 target_filename:="${HOME}/my_robot_calibration.yaml"

```

2. Run driver

```bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.102 headless_mode:=true launch_rviz:=false

```

3. Run moveit

```bash

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

```

4. Run ft driver

```bash

ros2 launch net_ft_driver net_ft_broadcaster.launch.py ip_address:=192.168.1.1 sensor_type:=onrobot

```

or

```bash

ros2 launch net_ft_driver net_ft_broadcaster.launch.py ip_address:=192.168.1.1 sensor_type:=onrobot namespace:=external_sensor

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

## Force - Based Movement Node

1. Launch simulation

```bash

python3 force_reflex.py

```

2. Run Wake up script in new terminal

```bash

ros2 run ur_tut_sim wake_up.py

```

3. Run Force Reflex script in another terminal

```bash

python3 force_reflex.py

```

4. Publish fake sensor forces (new terminal)

```bash
ros2 topic pub --once /force_torque_sensor_broadcaster/wrench geometry_msgs/msg/WrenchStamped "{header: {frame_id: 'tool0'}, wrench: {force: {x: 0.0, y: 0.0, z: 100.0}}}"
```

  ## Force - Based Movement on real robot

1. Run driver

```bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.102 headless_mode:=true launch_rviz:=false

```

2. Run ft driver (check if its working, if it doesn't restart)

```bash

ros2 launch net_ft_driver net_ft_broadcaster.launch.py ip_address:=192.168.1.1 sensor_type:=onrobot namespace:=external_sensor

```

3. Run Force Reflex script in another terminal

```bash

source ~/ur_energy_ws/install/setup.bash

python3 ~/ur_energy_ws/src/ur_tut_sim/scripts/real_reflex.py
```

## Surface_Centering / Spiral Script

  

1. Run Driver

2. Run FT Driver

3. Move to home position

```bash
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
  ],
  points: [
    {
      positions: [0.784, -1.105, 2.134, -2.600, -1.578, 0.009],
      time_from_start: {sec: 4}
    }
  ]
}"
```
  

4. Run Surface Centering script:

```bash

./surface_centering.py

```