## 使用介绍

下载依赖包

sudo apt install ros-melodic-turtlebot3*

sudo apt install ros-melodic-navigation

打开新终端,进入工程工作空间,依次输入：

source devel/setup.bash

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_house.launch     #第一次加载时间比较长，大概5分钟内

打开新终端,进入工程工作空间,依次输入：

source devel/setup.bash

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_navigation turtlebot3_navigation.launch 

纠正机器人位置

开始手动给点导航

打开相机拍照服务

rosrun take_photo camera_node 

开启行为树：

rosrun behaviortree_ros bt_action_point_node    (行为树越大，启动时间越久)



