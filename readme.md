realsenseから色付きのPointCloud2をPublishするには
```
ros2 launch catch_py rs.launch.py
```
現状ではトピックの宣言が入っているので，余裕があれば消すこと。

# ディレクトリ
### catch_py
今のところlaunchを記述するためだけにある。
### catch_cpp
- wrist_control_node
腕のDynamixel(mx106)を制御するノード
- finger_control_node
指のDynamixelを制御するノード
- read_serial_node
ESPから送られてくるシリアルを読み取ってPublishするノード

# 必要なパッケージ
```
sudo apt install ros-humble-dynamixel-sdk
```
realsense-rosが依存している
```
sudo apt install ros-humble-diagnostic-updater
```



```
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
sudo reboot
```

realsense-sdkはarmなので普通に入れても使えない。
```
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install realsense2
```