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
u sur 

ros2 run limo_nav_huy_test waypoints

```
ros2 run limo_nav_huy_test wandering
```


## Some command topic and parameters
ros2 topic echo /limo_status
