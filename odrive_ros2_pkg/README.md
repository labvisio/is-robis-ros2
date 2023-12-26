# odrive_ros2_pkg

## Run the project
```
sudo docker run --rm --privileged -it --network=host -v "$(pwd):/workspace/src" -v /dev/bus/usb:/dev/bus/usb --name=odrive_ros2 odrive_ros2_slan:v1 bash
```

## Initialize the odrive node
```
source install/setup.bash
cd src/odrive_ros2_pkg/launch
ros2 launch odrive_launch.py
```
## To consume the robot odometry 

```
sudo docker exec -it odrive_ros2 bash
```

```
source /opt/ros/humble/setup.bash
ros2 topic echo odrive/odom --field pose.pose.position
```
## To publish cmd_vel topic

```
sudo docker exec -it odrive_ros2 bash
```

```
source /opt/ros/humble/setup.bash
ros2 topic pub --once cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```




