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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

using namespace std::chrono_literals;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
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
const std::vector<FingerConfig> finger_config = {
    {1, 0.0, 2*M_PI, 0.0, false},
    {2, 0.0, 2*M_PI, 0.0, false},
    {3, 0.0, 2*M_PI, 0.0, false},
    {4, 0.0, 2*M_PI, 0.0, false},
    {5, 0.0, 2*M_PI, 0.0, false},
    {6, 0.0, 2*M_PI, 0.0, false},
    {7, 0.0, 2*M_PI, 0.0, false},
    {8, 0.0, 2*M_PI, 0.0, false}
};
const FingerConfig wrist_config = {9, 0.0, 2*M_PI, 0.0, false};
const char* DEVICE = "/dev/ttyUSB0";
const uint32_t BAUDRATE = 115200;
const int32_t current = 10;
const float load_limit = 70.0; // %
const float delta_rad = 0.1;
// ********************************************************************************************************************
// クラスの定義
// ********************************************************************************************************************
class DynamixelNode : public rclcpp::Node{
    private:
    rclcpp::Subscription<Float32MultiArray>::SharedPtr subscriber;
    rclcpp::Subscription<Float32>::SharedPtr wrist_deg_sub;
    rclcpp::TimerBase::SharedPtr timer;
    DynamixelWorkbench dxl_wb;
    std::vector<float> target_position;
    float wrist_target_pos = wrist_config.init;
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
            }else{
                std::cout << "検出されたモデル番号: " << model_number << std::endl;
            }
            // dxl_wb.currentBasedPositionMode(config.id, current, &log);
            dxl_wb.jointMode(config.id, 0, 0, &log);
            // std::cout << "ID " << (int)config.id << " を電流ベース位置制御モードに設定\n" << log << std::endl;
            std::cout << "ID " << (int)config.id << " をJointモードに設定\n" << log << std::endl;
        }

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
            for (int i = 0; i < finger_config.size(); i++){
                // モータにかかっている負荷を取得
                int32_t load;
                dxl_wb.itemRead(finger_config[i].id, "Present_Load", &load, &log);
                float load_percent = dxl_wb.convertValue2Load(load);
                if (load_percent > load_limit){
                    continue; // 負荷が設定値を超えている場合は動作しない
                }
                float target = target_position[i];
                if (finger_config[i].reverse){
                    target = -target;
                }
                target += finger_config[i].init;
                target = std::max(finger_config[i].min, std::min(finger_config[i].max, target));
                float present_position;
                dxl_wb.getRadian(finger_config[i].id, &present_position);
                if (target - present_position > delta_rad){
                    dxl_wb.goalPosition(finger_config[i].id, present_position + delta_rad, &log);
                }else if (target - present_position < -delta_rad){
                    dxl_wb.goalPosition(finger_config[i].id, present_position - delta_rad, &log);
                }else{
                    dxl_wb.goalPosition(finger_config[i].id, target, &log);
                }
            }
            float target = wrist_target_pos;
            if (wrist_config.reverse){
                target = -target;
            }
            target += wrist_config.init;
            target = std::max(wrist_config.min, std::min(wrist_config.max, target));
            dxl_wb.goalPosition(wrist_config.id, target, &log); 
        };
        
        auto topic_callback = [this](const Float32MultiArray::SharedPtr msg) -> void{
            for (int i = 0; i < msg->data.size(); i++){
                target_position[i] = msg->data[i]/180.0*M_PI;
            }
        };

        auto wrist_deg_callback = [this](const Float32::SharedPtr msg) -> void{
            wrist_target_pos = msg->data/180.0*M_PI;
        };

        int8_t qos_depth = 10;
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
        subscriber =this->create_subscription<Float32MultiArray>("x430_deg", QOS_RKL10V, topic_callback);
        wrist_deg_sub = this->create_subscription<Float32>("wrist_deg", QOS_RKL10V, wrist_deg_callback);
        for (auto config : finger_config){
            target_position.push_back(config.init);
        }
        timer = this->create_wall_timer(5ms, timer_callback);
    }

    ~DynamixelNode(){
        for (auto config : finger_config){
            dxl_wb.torqueOff(config.id, &log);
            std::cout << "ID " << (int)config.id << " のトルクをオフ\n" << log << std::endl;
        }
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
    auto node = std::make_shared<DynamixelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}