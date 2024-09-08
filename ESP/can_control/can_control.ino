#include "DJIMotorCtrlESP.hpp"
#include "cybergear_controller.hh"
#include "udp_read.hpp"
#include "cybergear_can_interface.hpp"

// テスト時の設定
const char* ssid = "NotFreeWiFi";
const char* password = "924865hirekatsu";
// 本番設定
// const char* ssid = "Kikaiken_WiFi";
// const char* password = "Kikaiken_WiFi";

// UDP通信設定
UDPRead udp(ssid, password);

// CAN settings
const uint8_t RX_PIN = 5;
const uint8_t TX_PIN = 4;

const uint8_t master_id = 0;
const std::vector<uint8_t> cyber_ids = {1, 2};
const int gm6020_1_id = 3;
const int gm6020_2_id = 4;
const int m3508_1_id = 5;
//                          kp,     ki,     kd,     dead_zone, max_value
pid_param m3508_speed(    5,      1,      0.01,   1,      10000);
pid_param m3508_location( 5,    0.00001,    10.0,   2000,   3000);
pid_param gm6020_speed(   10,     0,      0,      1,      16384);
pid_param gm6020_location(1.0,    0.1,    0.03,   5,      350);

M3508_P19 m3508_1(m3508_1_id, m3508_location, m3508_speed);
GM6020 gm6020_1(gm6020_1_id, gm6020_location, gm6020_speed);
GM6020 gm6020_2(gm6020_2_id, gm6020_location, gm6020_speed);
// Cybergear settings
CybergearController controller(master_id);
CybergearCanInterface interface;
//                              id, dir, limit_speed, limit_current, limit_torque, upper_position_limit, lower_position_limit, calib_direction, position_offset
CybergearSoftwareConfig config1(cyber_ids[0], CW, 30, 27, 12, 4.0*M_PI, -4.0*M_PI, CW, 0);
CybergearSoftwareConfig config2(cyber_ids[1], CW, 30, 27, 12, 4.0*M_PI, -4.0*M_PI, CW, 0);
std::vector<CybergearSoftwareConfig> sw_configs = {config1, config2};
std::vector<MotorStatus> motor_status;

bool calib_flag = true; // キャリブレーションフラグ

unsigned long reconstruct_id(uint8_t receive_can_id, uint8_t motor_can_id, uint8_t packet_type) {
    unsigned long reconstructed_id = 0;
    reconstructed_id |= receive_can_id; // 下位8ビットに receive_can_id を配置
    reconstructed_id |= ((unsigned long)motor_can_id << 8); // 次の8ビットに motor_can_id を配置
    reconstructed_id |= ((unsigned long)128 << 16); // 次の8ビットに unused_bits を配置
    reconstructed_id |= ((unsigned long)packet_type << 24); // 上位6ビットに packet_type を配置
    return reconstructed_id;
}

bool calib_motors(){
    float calib_torque = 0.2;
    float calib_speed = 0.01; // rad/s
    Serial.println("calib start");
    // トルク制御モードに変更
    controller.set_run_mode(MODE_CURRENT);
    delay(100);
    controller.send_current_command(cyber_ids, {calib_torque*sw_configs[0].calib_direction, calib_torque*sw_configs[1].calib_direction});
    delay(1000);
    bool flag1 = false;
    bool flag2 = false;
    while(true){ // 速度が一定以下になったら停止
        controller.get_motor_status(motor_status);
        Serial.println(motor_status[1].position*180/M_PI);
        controller.send_current_command(cyber_ids, {calib_torque*sw_configs[0].calib_direction, calib_torque*sw_configs[1].calib_direction});
        if(std::abs(motor_status[0].velocity) < calib_speed && !flag1){
            sw_configs[0].position_offset = -motor_status[0].position;
            controller.set_software_config(cyber_ids[0], sw_configs[0]);
            controller.send_current_command(cyber_ids[0], 0);
            Serial.println("motor 0 ok");
            flag1 = true;
        }
        if(std::abs(motor_status[1].velocity) < calib_speed && !flag2){
            sw_configs[1].position_offset = -motor_status[1].position;
            controller.set_software_config(cyber_ids[1], sw_configs[1]);
            controller.send_current_command(cyber_ids[1], 0);
            Serial.println("motor 1 ok");
            flag2 = true;
        }
        if(flag1 && flag2) {
            break;
        }
        delay(100);
    }
    // 位置制御モードに変更
    controller.set_run_mode(MODE_POSITION);
    Serial.println("calib finish");
    return true;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    adc_setup();
    udp.init();
    can_init(RX_PIN, TX_PIN, 1000);
    controller.init(cyber_ids, sw_configs, MODE_POSITION, &interface, 0);
    for(auto id : cyber_ids){
        controller.set_speed_limit(id, 30);
        controller.set_torque_limit(id, 12);
        controller.set_current_limit(id, 27);
        controller.set_position_control_gain(id, 30); // 30
        controller.set_velocity_control_gain(id, 1.0, 0.002); // 1.0, 0.002)
        controller.set_current_control_param(id, 0.125, 0.0158, 0.1); // 0.125, 0.0158, 0.1
        add_user_can_func(reconstruct_id(master_id, id, CMD_RESPONSE), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
        // add_user_can_func(reconstruct_id(master_id, id, CMD_RAM_READ), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
        // add_user_can_func(reconstruct_id(master_id, id, CMD_GET_MOTOR_FAIL), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
    }
    m3508_1.setup();
    gm6020_1.setup();
    gm6020_2.setup();
    controller.enable_motors();
    // calib_motors();
    Serial.println("setup finish");
}

std::vector<float> target_angle = {0, 0, 0, 0, 0};
uint8_t mode;
void loop() {
    mode = udp.get_mode();
    if (mode == 0) { // 接続テスト
        Serial.println(2);
    }
    else if (mode == 1) { // 位置制御
        Serial.println(3);
        udp.get_data(target_angle); // degree
        // CAN ID順に割り当てる
        controller.send_position_command(cyber_ids, {target_angle[0]*M_PI/180, target_angle[1]*M_PI/180});
        gm6020_1.set_angle(target_angle[2]);
        gm6020_2.set_angle(target_angle[3]);
        m3508_1.set_angle(target_angle[4]);
    }
    else if (mode == 2) { // サスペンド
        Serial.println(4);
    }
    delay(10);
}