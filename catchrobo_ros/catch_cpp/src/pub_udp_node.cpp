#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cstring>      // for memset
#include <sys/socket.h> // for socket functions
#include <arpa/inet.h>  // for inet_addr
#include <unistd.h>     // for close

using namespace std::chrono_literals;
using Int32 = std_msgs::msg::Int32;
using Twist = geometry_msgs::msg::Twist;

const char* ipAddress = "192.168.0.77";
int port = 9999;

class PubUDPNode : public rclcpp::Node {
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<Twist>::SharedPtr twist_sub;
    rclcpp::Subscription<Int32>::SharedPtr mode_sub;
    float target_position[5];
    uint8_t mode = 0;
    struct sockaddr_in serverAddress;
    int sock;

    public:
    PubUDPNode() : Node("pub_udp_node") {
        std::cout << "call PubUDPNode!" << std::endl;
        auto timer_callback = [this]() -> void {
            uint8_t buffer[sizeof(mode) + sizeof(float) * 5];
            memset(buffer, 0, sizeof(buffer));
            memcpy(buffer, &mode, sizeof(mode)); 
            memcpy(buffer + sizeof(mode), target_position, sizeof(float) * 5); // その後にfloat 5つ
            ssize_t sentBytes = sendto(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
            if (sentBytes < 0) {
                perror("Failed to send data");
            } else {
                std::cout << "Data sent successfully.\n";
            }
        };
        
        auto twist_callback = [this](const Twist::SharedPtr msg) -> void {
            target_position[0] = (float)msg->linear.x;
            target_position[1] = (float)msg->linear.y;
            target_position[2] = (float)msg->linear.z;
            target_position[3] = (float)msg->angular.x;
            target_position[4] = (float)msg->angular.y;
        };

        auto mode_callback = [this](const Int32::SharedPtr msg) -> void {
            mode = msg->data;
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
        mode_sub = this->create_subscription<Int32>("mode", 10, mode_callback);
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
