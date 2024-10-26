// xl430 + mx106

// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <cstdio>
#include <memory>
#include <string>
#include <map>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "../include/dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

using namespace std::chrono_literals;
using Quaternion = geometry_msgs::msg::Quaternion;
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
    {1, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {2, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {3, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {4, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {5, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {6, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {7, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {8, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {9, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {10, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {11, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {12, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {13, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {14, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {15, -179*M_PI/180, 179*M_PI/180, 0.0, false},
    {16, -179*M_PI/180, 179*M_PI/180, 0.0, false},
};
const char* DEVICE = "/dev/U2D2-1";
const uint32_t BAUDRATE = 1000000; // 1Mbps
const float CURRENT = 50.0; // 40mA以下だと動かない
// ********************************************************************************************************************
// クラスの定義
// ********************************************************************************************************************
class DynamixelNode : public rclcpp::Node{
    private:
    rclcpp::Subscription<Quaternion>::SharedPtr sub1;
    rclcpp::Subscription<Quaternion>::SharedPtr sub2;
    rclcpp::Subscription<Quaternion>::SharedPtr sub3;
    rclcpp::Subscription<Quaternion>::SharedPtr sub4;
    rclcpp::Publisher<Float32MultiArray>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    DynamixelWorkbench dxl_wb;
    std::vector<float> target = std::vector<float>(16, 0.0);
    std::vector<float> now_angle = std::vector<float>(16, 0.0);
    const char* log;

    public:
    DynamixelNode() : Node("dynamixel_node"){
        RCLCPP_INFO(this->get_logger(), "dynamixel_node is activated");
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
            }else if(model_number == XL330_M288){
                std::cout << "検出されたモデル: XL_330" << std::endl;
            }else{
                std::cout << "検出されたモデル番号: " << model_number << std::endl;
            }
            dxl_wb.currentBasedPositionMode(config.id, dxl_wb.convertCurrent2Value(CURRENT), &log);
        }
        std::cout << "セットアップが完了しました\n" << std::endl;

        auto timer_callback = [this]() -> void{
            for (int i = 0; i < finger_config.size(); i++){
                float target_ = target[i];
                if (finger_config[i].reverse) target_ = -target_;
                target_ += finger_config[i].init;
                target_ = std::max(finger_config[i].min, std::min(finger_config[i].max, target_));
                dxl_wb.goalPosition(finger_config[i].id, target_, &log);
                dxl_wb.getRadian(finger_config[i].id, &now_angle[i], &log);
                now_angle[i] = (now_angle[i] - finger_config[i].init) * (finger_config[i].reverse ? -1 : 1) * 180.0/M_PI;
            }
            auto msg = Float32MultiArray();
            msg.data = now_angle;
            pub->publish(msg);
        };
        
        auto topic_callback1 = [this](const Quaternion::SharedPtr msg) -> void{
            target[0] = msg->x/180.0*M_PI;
            target[1] = msg->y/180.0*M_PI;
            target[2] = msg->z/180.0*M_PI;
            target[3] = msg->w/180.0*M_PI;
        };

        auto topic_callback2 = [this](const Quaternion::SharedPtr msg) -> void{
            target[4] = msg->x/180.0*M_PI;
            target[5] = msg->y/180.0*M_PI;
            target[6] = msg->z/180.0*M_PI;
            target[7] = msg->w/180.0*M_PI;
        };

        auto topic_callback3 = [this](const Quaternion::SharedPtr msg) -> void{
            target[8] = msg->x/180.0*M_PI;
            target[9] = msg->y/180.0*M_PI;
            target[10] = msg->z/180.0*M_PI;
            target[11] = msg->w/180.0*M_PI;
        };

        auto topic_callback4 = [this](const Quaternion::SharedPtr msg) -> void{
            target[12] = msg->x/180.0*M_PI;
            target[13] = msg->y/180.0*M_PI;
            target[14] = msg->z/180.0*M_PI;
            target[15] = msg->w/180.0*M_PI;
        };

        int8_t qos_depth = 10;
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
        sub1 = this->create_subscription<Quaternion>("finger1_deg", QOS_RKL10V, topic_callback1);
        sub2 = this->create_subscription<Quaternion>("finger2_deg", QOS_RKL10V, topic_callback2);
        sub3 = this->create_subscription<Quaternion>("finger3_deg", QOS_RKL10V, topic_callback3);
        sub4 = this->create_subscription<Quaternion>("finger4_deg", QOS_RKL10V, topic_callback4);
        pub = this->create_publisher<Float32MultiArray>("finger_deg", QOS_RKL10V);
        for (int i = 0; i < finger_config.size(); i++){
            target[i] = finger_config[i].init;
        }
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    ~DynamixelNode(){
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
    auto node = std::make_shared<DynamixelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}