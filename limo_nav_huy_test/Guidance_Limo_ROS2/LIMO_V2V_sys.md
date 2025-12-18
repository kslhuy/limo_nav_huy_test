## Remote desktop connection

#### 3. Connect to Limo remotely

Select connection object (double click)

![](./Limo_image/nomachine1.png)

<font color=blue>Username：agilex       

Password：agx </font>
 
Select to save the password.

Always select the default option : OK.



### Navigation

## Navigation 1. LiDAR Mapping :  need a map prebuild , so we get that by using lidar

Cartographer mapping :First, start the LiDAR. Launch a new terminal and enter the command:

```
ros2 launch limo_bringup limo_start.launch.py
```

Then start the cartographer mapping algorithm. Open another new terminal and enter the command:

```
ros2 launch limo_bringup limo_cartographer.launch.py
```

After building the map, it is necessary to save it. Three following commands need to be entered in the terminal:

```
ros2 run nav2_map_server map_saver_cli -f name_of_the_map
```


## Usage

### Build the Package:
```bash
cd ~/agilex_ws
colcon build --packages-select limo_nav_huy_test --symlink-install
source install/setup.bash
```

### Run the Node:
```bash
ros2 run limo_nav_huy_test vehicle_main_ros
```

### With Parameters:
```bash
ros2 run limo_nav_huy_test vehicle_main_ros \
  --ros-args \
  -p car_id:=0 \
  -p v_ref:=0.5 \
  -p controller_rate:=100
```


### Build - and test Sysem multi vehicle V2V package 
```
colcon build --packages-select limo_nav_huy_test
```
run the scritp
```
ros2 launch limo_bringup limo_start.launch.py
```

```
ros2 launch limo_bringup limo_nav2.launch.py
```

```
ros2 run limo_nav_huy_test vehicle_main_ros
```




## Some command Run NODE 
Run Rivz
```
ros2 run rviz2 rviz2
```

```
ros2 run limo_nav_huy_test waypoints
```

```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=bib_cran_map.yaml 

ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:=['map_server']
```

- simple controller 
```
ros2 run limo_nav_huy_test wandering
```


## Some command topic and parameters
ros2 topic echo /limo_status

Aligned the path with the map 
ros2 param set /waypoints mirror_x True

ros2 param set /waypoints theta_deg 80.0

ros2 param set /waypoints dy 0.0




ros2 param set /waypoints scale 0.9
