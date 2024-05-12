#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::PoseStamped;

std::string port = "/dev/ttyUSB0";

class QuatNode : public rclcpp::Node{
public:
  QuatNode() : Node("quat_node"){
    std::cout << "call QuatNode!" << std::endl;
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    publisher = this->create_publisher<Pose>("quaternion", qos);

    serialData.open(port);
    while(!serialData.is_open() && rclcpp::ok()){
      std::cout << "シリアルポート(" << port << ")を開けませんでした" << std::endl;
      std::this_thread::sleep_for(1s);
      serialData.open(port);
    }
    std::string command = "stty -F " + port + " 115200";
    system(command.c_str());

    timer = this->create_wall_timer(10ms, [this]() -> void {
      auto message = Pose();
      std::string line;
      if (std::getline(serialData, line)) {
        std::cout << "data [" << line << "]" << std::endl;
        if(line.size() >= 19){
          std::istringstream iss(line);
          std::string value;
          int count = 0;
          while (std::getline(iss, value, ',')) {
            if(value.size() >= 4){
              double data = std::stod(value);
              switch (count) {
                case 0:
                  message.pose.orientation.w = data;
                  break;
                case 1:
                  message.pose.orientation.x = data;
                  break;
                case 2:
                  message.pose.orientation.y = data;
                  break;
                case 3:
                  message.pose.orientation.z = data;
                  break;
                default:
                  break;
              }
              count++;
            }
          }
          std::cout << message.pose.orientation.w << " " << message.pose.orientation.x << " " << message.pose.orientation.y << " " << message.pose.orientation.z << std::endl;
          message.header.stamp = this->now();
          message.header.frame_id = "map";
          publisher->publish(message);
        }
      }

    });
  }

  ~QuatNode(){
    serialData.close();
  }

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<Pose>::SharedPtr publisher;
  std::ifstream serialData;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<QuatNode>());
  rclcpp::shutdown();
  return 0;
}