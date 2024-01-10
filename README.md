# Robis ros2

## Odrive Ros2

### Run the project
```
sudo docker run --rm --privileged -it --network=host -v "$(pwd):/workspace/src" -v /dev/bus/usb:/dev/bus/usb --name=robis_ro2 matheusdutra0207/is-robis-ros2:0.0.3 bash
```

### Initialize the odrive node
```
source install/setup.bash
ros2 launch odrive_ros2_pkg is_robis_ros2_launch.py
```
### Consume the robot odometry 

```
sudo docker exec -it robis_ro2 bash
```

```
source /opt/ros/humble/setup.bash
ros2 topic echo odrive/odom --field pose.pose.position
```
### Publish cmd_vel topic

```
sudo docker exec -it robis_ro2 bash
```

```
source /opt/ros/humble/setup.bash
ros2 topic pub --once cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

### Map an environment

```
sudo docker exec -it robis_ro2 bash
```
#### Edit the config file
```
vim opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```
#### Run the Slam Toolbox
```
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```


