# ros2 foxy packages for omorobot r1d2

This project is to demonstrate r1d2 control and navigation in ros2-foxy environment.


## Install ros2 foxy

### Clone source

```bash
  cd ~
  git clone https://github.com/t-shaped-person/quick-ros2-setup
```

### Run script

```bash
  cd ~/quick-ros2-setup
  ./1_ros2_foxy_install.sh
```


## Make workspace and install packages

### Clone source

```bash
  cd ~
  git clone https://github.com/t-shaped-person/quick-ros2-setup
```

### Run script for workspace and packages

```bash
  cd ~/quick-ros2-setup
  ./2_foxy_ws_setup_r1d2.sh
```

### Run script for udev rules

```bash
  cd ~/quick-ros2-setup
  sudo ./3_udev_rules_r1d2.sh
```


## Play with the robot

- To bringup robot

```bash
cd {$workspace_path}
ros2 launch omo_r1_bringup omo_r1_bringup.launch.py
```

- To teleoperate the robot using **KEYBOARD**

```bash
cd {$workspace_path}
ros2 run omo_r1_teleop teleop_keyboard
```

- To conduct SLAM (Try after few seconds from MCU and LiDAR bringup)

```bash
cd {$workspace_path}
ros2 launch omo_r1_cartographer cartographer.launch.py
ros2 launch omo_r1_cartographer cartographer_rviz.launch.py
```

- Once mapping is done, you can create map.pgm and map.yaml file by executing

```bash
cd {$HOME}
ros2 run nav2_map_server map_saver_cli -f map
```

- To conduct path planning & following (Try after few seconds from MCU and LiDAR bringup)
```bash
cd {$workspace_path}
ros2 launch omo_r1_navigation2 navigation2.launch.py map:=$HOME/map.yaml
ros2 launch omo_r1_navigation2 navigation2_rviz.launch.py
```