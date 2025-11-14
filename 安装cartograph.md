# 安装Cartographer

[TOC]

# 1 安装cartograph

## 1.1 [安装ROS2](https://so.csdn.net/so/search?q=安装ROS2&spm=1001.2101.3001.7020) Humble

```shell
sudo apt update
sudo apt install ros-humble-desktop
```



## 1.2 安装Gazebo11

```shell
sudo apt install gazebo11 libgazebo11-dev
```



## 1.3 安装Cartographer ROS2

### 安装依赖

```shell
sudo apt update
sudo apt install -y python3-wstool python3-rosdep ninja-build stow
```

### 创建工作空间

```shell
mkdir -p ~/cartographer_ws/src
cd ~/cartographer_ws/src
```

### 克隆Cartographer相关代码

```shell
git clone https://github.com/ros2/cartographer.git -b ros2
git clone https://github.com/ros2/cartographer_ros.git -b ros2

//git clone卡住可以直接去github官方网址下载压缩包
```

### 安装依赖

```shell
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro humble -y

//ROS 的一个依赖管理工具，自动为 ROS 工作区安装系统依赖包（如 apt 包）
//
--from-paths .	从当前目录（即你的工作区，比如 src/ 下的所有包）扫描 package.xml，找出需要的依赖。
--ignore-src	忽略源码包本身（也就是你自己已经放在工作区 src 里的包，不需要重复安装）。
--rosdistro humble	指定 ROS 发行版（这里是 Humble）。确保安装与该版本匹配的依赖。
-y	自动确认安装（不用手动输入 “yes”）。

//rosdep执行前需要先初始化
sudo rosdep init
```

### 构建和安装

```shell
cd ~/cartographer_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

//
colcon build	使用 colcon 编译整个 ROS2 工作区
--symlink-install	使用符号链接方式安装（便于开发调试）
--cmake-args -DCMAKE_BUILD_TYPE=Release	给 CMake 传参，设定为 Release 模式（优化性能）
```

### 配置三维SLAM

在`cartographer_ros/configuration_files`目录下创建新的配置文件`my_robot_3d.lua`

```lua
-- ...existing code...
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

trajectory_builder_3d = TRAJECTORY_BUILDER_3D
trajectory_builder_3d.submaps.num_range_data = 100
trajectory_builder_3d.min_range = 1.
trajectory_builder_3d.max_range = 100.
trajectory_builder_3d.motion_filter.max_angle_radians = math.rad(0.5)

return options
-- ...existing code...
```

# 2. 创建启动文件

创建新的启动文件`my_robot_3d.launch.py`：

```lua
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', 
                os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files'),
                '-configuration_basename', 'my_robot_3d.lua'
            ],
            remappings=[
                ('points2', '/camera/depth/points'),
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])

```

# 在Gazebo中运行仿真

#### 1. 启动Gazebo仿真环境



#### 2. 启动Cartographer

```shell
source ~/cartographer_ws/install/setup.bash
ros2 launch cartographer_ros my_robot_3d.launch.py
```

**解决**

cartographer_ros/CMakeLists.txt

```cmake
pkg_search_module(CAIRO REQUIRED cairo>=1.12.16)

# Library
#add_library(${PROJECT_NAME} SHARED
add_library(${PROJECT_NAME} STATIC
  src/assets_writer.cpp
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
```

cartographer_rviz/CMakeLists.txt

```cmake
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2_ros REQUIRED)#
find_package(tf2_eigen REQUIRED)#
find_package(pcl_conversions REQUIRED)#
find_package(urdf REQUIRED)#


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)



target_link_libraries(${PROJECT_NAME} PUBLIC
  absl::synchronization
  cartographer
  cartographer_ros::cartographer_ros
  Eigen3::Eigen
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen #
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  pluginlib::pluginlib
```



#### 3. 启动RViz查看地图

```shell
ros2 run rviz2 rviz2 -d ~/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files/demo_3d.rviz
```



### 生成八叉树地图

#### 1. 安装octomap包

```
sudo apt install ros-humble-octomap-server
```

#### 2. 创建八叉树地图生成节点

创建`octomap_generator.py`：

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap
import octomap

class OctomapGenerator(Node):
    def __init__(self):
        super().__init__('octomap_generator')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/map',
            self.pointcloud_callback,
            10)
        self.octomap_pub = self.create_publisher(Octomap, '/octomap', 10)
        
        # 创建八叉树地图，分辨率为0.05米
        self.octree = octomap.OcTree(0.05)
        
    def pointcloud_callback(self, msg):
        # 将PointCloud2转换为八叉树地图
        # 这里需要实现PointCloud2到octomap的转换
        # 实际实现会更复杂，需要处理点云数据格式
        
        # 发布八叉树地图
        octomap_msg = Octomap()
        octomap_msg.header.frame_id = "map"
        octomap_msg.binary = True
        # 将octree序列化到消息中
        # 实际实现需要将octree转换为消息格式
        
        self.octomap_pub.publish(octomap_msg)
        
def main(args=None):
    rclpy.init(args=args)
    octomap_generator = OctomapGenerator()
    rclpy.spin(octomap_generator)
    octomap_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 保存地图

```shell
# 先停止建图：
ros2 service call /finish_trajectory 0

# 然后保存成pbstream，自己更改保存路径，注意不能写相对路径，==
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{
  filename: '${HOME}/mymap.pbstream',
  include_unfinished_submaps: false
}"

#导出成一般导航使用的yaml+pgm
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename ${HOME}/mymap.pbstream \
  -map_filestem ${HOME}/mymap \
  -resolution 0.05
```



#### 1. 保存三维点云地图

```shell
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -map_filestem=/tmp/my_map \
  -pbstream_filename=/tmp/my_map.pbstream \
  -resolution=0.05
```

#### 2. 保存八叉树地图

```shell
ros2 service call /octomap_binary get_octomap
#ros2 service call /octomap_binary octomap_msgs/srv/GetOctomap "{}"
# 然后使用适当的工具保存接收到的八叉树地图

sudo apt install ros-humble-octomap-msgs  # 以 Humble 版本为例，替换为你的 ROS 2 版本

```

注意事项
确保Gazebo中的传感器配置正确，特别是深度相机的参数
根据实际机器人模型调整Cartographer的配置文件
八叉树地图生成需要额外的点云处理，可能需要使用PCL库
对于大型环境，可能需要调整Cartographer的参数以提高性能和精度

故障排除
如果遇到依赖问题，确保所有必要的ROS2包都已安装
如果Gazebo无法启动，检查Gazebo安装和ROS2集成
如果Cartographer无法接收数据，检查话题名称和 remappings
