# 森林狼战队-rm_2025——serial

## 一. 环境搭建与编译

```sh
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-serial-driver
```

```sh
git clone https://github.com/Xiancaijiang/rm_2025_serial.git
cd rm_2025_serial
```

```sh
colcon build --symlink-install
```

## 二. 使用帮助

### 1.1 修改，添加决策所需信息
- GAME_STATUS
- ROBOT_STATU
- SHOOT_DATA


### 1.2 启动可视化

```sh
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### 1.3 单独运行子模块


- 串口模块

    ```sh
    sudo chmod 777 /dev/ttyACM0

    source install/setup.bash
    ros2 launch rm_serial_driver serial_driver.launch.py
    ```

- 读取裁判系统模块

    ```sh
    sudo chmod 777 /dev/ttyACM1

    source install/setup.bash
    ros2 run rm_referee_ros2 rm_referee_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200 -p debug_mode:=true
    ```


## 三. 相关信息

### 3.1 通讯协议

详见：[README (rm_serial_driver)](/src/rm_serial_driver/README.md)

## 其他文档

> 本项目迁移前为北极熊视觉系统，有基于 ROS2 实现不同兵种所需的视觉功能，可以继续添加扩展。

rm_vision 部署文档： [部署华师视觉项目](https://flowus.cn/lihanchen/share/0d472992-f136-4e0e-856f-89328e99c684) 

测算相机畸变与内参矩阵：[相机标定](https://flowus.cn/lihanchen/share/02a518a0-f1bb-47a5-8313-55f75bab21b5)
