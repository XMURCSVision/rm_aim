## rm_vision部署

0. 安装ros2-humble

根据下面指令的提示一键安装ros2-humble桌面版

```
wget http://fishros.com/install -O fishros && . fishros
```

1. 安装依赖

依赖安装方式有两种，二选一

```
# 方法一，手动安装每个包
sudo apt install \
  ros-humble-io-context \
  ros-humble-serial-driver \
  ros-humble-udp-msgs \
  ros-humble-ament-clang-format \
  ros-humble-ament-cmake-clang-format \
  ros-humble-compressed-depth-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-theora-image-transport \
  ros-humble-camera-calibration-parsers \
  ros-humble-camera-info-manager \
  ros-humble-camera-calibration \
  ros-humble-vision-opencv \
  ros-humble-xacro \
  ros-humble-asio-cmake-module -y

# 方法二，rosdep自动安装，但需要好的网络(翻墙)
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y

sudo apt install ros-humble-asio-cmake-module -y
```

2. 编译

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache
```
