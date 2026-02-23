# ur_servo_keyboard_input
Using ROS2 API to interact with moveit servo

## Load controller
```bash
ros2 control load_controller --set-state inactive forward_position_controller
```

## Switch controller
```bash
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller
```


## Run keyboard servo input (beta)
```bash
ros2 run ur_servo_keyboard_input keyboard_joint_jog
```