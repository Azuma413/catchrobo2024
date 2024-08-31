// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <cstdio>
#include <memory>
#include <string>
#include <map>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "../include/dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "std_msgs/msg/float32_multi_array.hpp"

#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

using namespace std::chrono_literals;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
struct FingerConfig{
    uint8_t id;
    float min;
    float max;
    float init; // 初期位置=指がまっすぐのときの角度。この値を0とする。
    bool reverse;
};
// ********************************************************************************************************************
// 定数等の定義
// ********************************************************************************************************************
const std::vector<FingerConfig> finger_config = {
    {1, 0.0, 2*M_PI, 0.0, false},
    {2, 0.0, 2*M_PI, 0.0, false},
    {3, 0.0, 2*M_PI, 0.0, false},
    {4, 0.0, 2*M_PI, 0.0, false},
    {5, 0.0, 2*M_PI, 0.0, false},
    {6, 0.0, 2*M_PI, 0.0, false},
    {7, 0.0, 2*M_PI, 0.0, false},
    {8, 0.0, 2*M_PI, 0.0, false},
    {9, 0.0, 2*M_PI, 0.0, false},
    {10, 0.0, 2*M_PI, 0.0, false},
    {11, 0.0, 2*M_PI, 0.0, false},
    {12, 0.0, 2*M_PI, 0.0, false},
    {13, 0.0, 2*M_PI, 0.0, false},
    {14, 0.0, 2*M_PI, 0.0, false},
    {15, 0.0, 2*M_PI, 0.0, false},
    {16, 0.0, 2*M_PI, 0.0, false}
};
const char* DEVICE = "/dev/ttyUSB0";
const uint32_t BAUDRATE = 115200; //1000000;
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class FingerControlNode : public rclcpp::Node{
    private:
    rclcpp::Subscription<Float32MultiArray>::SharedPtr subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    DynamixelWorkbench dxl_wb;
    std::vector<float> target_position;
    const char* log;

    public:
    FingerControlNode() : Node("finger_control_node"){
        RCLCPP_INFO(this->get_logger(), "finger_control_node is activated");
        std::cout << "セットアップを開始します\n" << std::endl;

        dxl_wb.init(DEVICE, BAUDRATE, &log);
        std::cout << "シリアルポートの初期化(" << DEVICE << ", " << BAUDRATE << ")\n" << log << std::endl;

        uint16_t model_number;
        for (auto config : finger_config){
            dxl_wb.ping(config.id, &model_number, &log);
            std::cout << "ID " << (int)config.id << " へPingを送信\n" << log << std::endl;
            if(model_number == PRO_H42_20_S300_R_A){
                std::cout << "検出されたモデル: PRO_H42_20_S300_R_A" << std::endl;
            }else if(model_number == MX_106_2){
                std::cout << "検出されたモデル: MX_106_2" << std::endl;
            }else if(model_number == XM430_W350){
                std::cout << "検出されたモデル: XM430_W350" << std::endl;
            }else if(model_number == XL_320){
                std::cout << "検出されたモデル: XL_320" << std::endl;
            }else if(model_number == XL430_W250){
                std::cout << "検出されたモデル: XL_430" << std::endl;
            }else{
                std::cout << "検出されたモデル番号: " << model_number << std::endl;
            }

            dxl_wb.jointMode(config.id, 0, 0, &log);
            std::cout << "ID " << (int)config.id << " をジョイントモードに設定\n" << log << std::endl;
        }
        std::cout << "セットアップが完了しました\n" << std::endl;

        auto timer_callback = [this]() -> void{
            for (int i = 0; i < finger_config.size(); i++){
                float target = target_position[i];
                if (finger_config[i].reverse){
                    target = -target;
                }
                dxl_wb.goalPosition(finger_config[i].id, target + finger_config[i].init, &log);
            }
        };
        
        auto topic_callback = [this](const Float32MultiArray::SharedPtr msg) -> void{
            for (int i = 0; i < msg->data.size(); i++){
                target_position[i] = msg->data[i];
            }
        };

        this->declare_parameter("qos_depth", 10);
        int8_t qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
        subscriber =this->create_subscription<Float32MultiArray>("finger_rad", QOS_RKL10V, topic_callback);
        for (auto config : finger_config){
            target_position.push_back(config.init);
        }
        timer = this->create_wall_timer(5ms, timer_callback);
    }

    ~FingerControlNode(){
        for (auto config : finger_config){
            dxl_wb.torqueOff(config.id, &log);
            std::cout << "ID " << (int)config.id << " のトルクをオフ\n" << log << std::endl;
        }
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FingerControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}