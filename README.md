# inverted_pendulum_control

```
ros2 launch inverted_pendulum_control inverted_pendulum_control_bringup.launch.py start_rviz:=true

ros2 topic pub --rate 30 /inverted_pendulum_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.3
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"

ros2 topic echo /odom
```