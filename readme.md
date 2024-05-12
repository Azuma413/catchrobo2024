realsenseから色付きのPointCloud2をPublishするには
```
ros2 launch catch_py rs.launch.py
```
現状ではトピックの宣言が入っているので，余裕があれば消すこと。


# 必要なパッケージ
```
sudo apt install ros-humble-librealsense2*
```
```
sudo apt install ros-humble-dynamixel-sdk
```
realsense-rosが依存している
```
sudo apt install ros-humble-diagnostic-updater
```