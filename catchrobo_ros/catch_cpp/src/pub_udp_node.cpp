#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cstring>      // for memset
#include <sys/socket.h> // for socket functions
#include <arpa/inet.h>  // for inet_addr
#include <unistd.h>     // for close

using namespace std::chrono_literals;
using Bool = std_msgs::msg::Bool;
using Twist = geometry_msgs::msg::Twist;

const char* ipAddress = "192.168.0.77";
int port = 9999;

class PubUDPNode : public rclcpp::Node {
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<Twist>::SharedPtr twist_sub;
    rclcpp::Subscription<Bool>::SharedPtr bool_sub;
    std::vector<double> target_position = std::vector<double>(5, 0.0);  // 初期化
    bool flag = false;
    struct sockaddr_in serverAddress;
    int sock;

    public:
    PubUDPNode() : Node("pub_udp_node") {
        std::cout << "call PubUDPNode!" << std::endl;
        auto timer_callback = [this]() -> void {
            uint8_t boolByte = static_cast<uint8_t>(flag);
            float floats[5] = {
                static_cast<float>(target_position[0]), 
                static_cast<float>(target_position[1]), 
                static_cast<float>(target_position[2]), 
                static_cast<float>(target_position[3]), 
                static_cast<float>(target_position[4])
            };
            uint8_t buffer[sizeof(boolByte) + sizeof(float) * 5];
            memset(buffer, 0, sizeof(buffer));
            memcpy(buffer, &boolByte, sizeof(boolByte)); 
            memcpy(buffer + sizeof(boolByte), floats, sizeof(float) * 5); // その後にfloat 5つ

            ssize_t sentBytes = sendto(sock, buffer, sizeof(buffer), 0, 
                                       (struct sockaddr*)&serverAddress, sizeof(serverAddress));
            if (sentBytes < 0) {
                perror("Failed to send data");
            } else {
                std::cout << "Data sent successfully.\n";
            }
        };
        
        auto twist_callback = [this](const Twist::SharedPtr msg) -> void {
            target_position = {msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y};
        };

        auto bool_callback = [this](const Bool::SharedPtr msg) -> void {
            flag = msg->data;
        };

        // udp setup
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "Failed to create socket.\n";
            exit(EXIT_FAILURE);  // 変更
        }
        memset(&serverAddress, 0, sizeof(serverAddress));
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        serverAddress.sin_addr.s_addr = inet_addr(ipAddress);

        twist_sub = this->create_subscription<Twist>("target", 10, twist_callback);
        bool_sub = this->create_subscription<Bool>("flag", 10, bool_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    ~PubUDPNode() {
        close(sock);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubUDPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
