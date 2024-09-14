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
#include "std_msgs/msg/float32.hpp"

#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

using namespace std::chrono_literals;
using Float32 = std_msgs::msg::Float32;
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
const FingerConfig wrist_config = {10, -80*M_PI/180, 100*M_PI/180, 10*M_PI/180, true};
const char* DEVICE = "/dev/U2D2-0";
const uint32_t BAUDRATE = 115200;
const float load_limit = 90.0; // %
// ********************************************************************************************************************
// クラスの定義
// ********************************************************************************************************************
class WristNode : public rclcpp::Node{
    private:
    rclcpp::Subscription<Float32>::SharedPtr wrist_deg_sub;
    rclcpp::TimerBase::SharedPtr timer;
    DynamixelWorkbench dxl_wb;
    float wrist_target_pos = 0;
    const char* log;

    public:
    WristNode() : Node("wrist_node"){
        RCLCPP_INFO(this->get_logger(), "wrist_node is activated");
        std::cout << "セットアップを開始します\n" << std::endl;

        dxl_wb.init(DEVICE, BAUDRATE, &log);
        std::cout << "シリアルポートの初期化(" << DEVICE << ", " << BAUDRATE << ")\n" << log << std::endl;

        uint16_t model_number;
        dxl_wb.ping(wrist_config.id, &model_number, &log);
        std::cout << "ID " << (int)wrist_config.id << " へPingを送信\n" << log << std::endl;
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
        dxl_wb.jointMode(wrist_config.id, 0, 0, &log);
        std::cout << "ID " << (int)wrist_config.id << " をJointモードに設定\n" << log << std::endl;
        std::cout << "セットアップが完了しました\n" << std::endl;

        auto timer_callback = [this]() -> void{
            float wrist_target = wrist_target_pos;
            if (wrist_config.reverse){
                wrist_target = -wrist_target;
            }
            wrist_target += wrist_config.init;
            wrist_target = std::max(wrist_config.min, std::min(wrist_config.max, wrist_target));
            dxl_wb.goalPosition(wrist_config.id, wrist_target, &log); 
            // std::cout << "set pos: "<< log << std::endl;
        };
        
        auto wrist_deg_callback = [this](const Float32::SharedPtr msg) -> void{
            wrist_target_pos = msg->data/180.0*M_PI;
            // std::cout << "get target: " << msg->data << std::endl;
        };

        int8_t qos_depth = 10;
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
        wrist_deg_sub = this->create_subscription<Float32>("wrist_deg", QOS_RKL10V, wrist_deg_callback);
        timer = this->create_wall_timer(2ms, timer_callback);
    }

    ~WristNode(){
        dxl_wb.torqueOff(wrist_config.id, &log);
        std::cout << "ID " << (int)wrist_config.id << " のトルクをオフ\n" << log << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WristNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}