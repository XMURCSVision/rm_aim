## rm_vision部署

- 安装ros2 humble
- 初始化rosdep

```
sudo rosdep init
rosdep update
# 这一步和网络有关系
```

- 安装packages

```
# 在根目录下，rosdep会根据每个pkg的package.xml文件下载需要的pkg
rosdep install --from-paths src --ignore-src -r -y
```

- 编译

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
