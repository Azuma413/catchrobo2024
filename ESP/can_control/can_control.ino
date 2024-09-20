#include "DJIMotorCtrlESP.hpp"
#include "cybergear_controller.hh"
#include "udp_read.hpp"
#include "cybergear_can_interface.hpp"

// テスト時の設定
// const char* ssid = "NotFreeWiFi";
// const char* password = "924865hirekatsu";
// 本番設定
const char* ssid = "Kikaiken_WiFi";
const char* password = "Kikaiken_WiFi";

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
pid_param m3508_location( 5,    0.0,    10.0,   2000,   3000);
// pid_param m3508_location( 5,    0.00001,    10.0,   2000,   3000);
pid_param gm6020_speed(   10,     0,      0.01,      1,      16384);
pid_param gm6020_location1(2.0,    0.001,    0.05,   5,      350);
pid_param gm6020_location2(1.0,    0.1,    0.03,   5,      350);
            // pid_param speed_pid_parmater(10,0,0,1,16384);
            // pid_param location_pid_parmater(0.2,0.1,0,5,350);
            // pid_param gm6020_speed(   10,     0,      0,      1,      16384);
            // pid_param gm6020_location(1.0,    0.1,    0.03,   5,      350);

M3508_P19 m3508_1(m3508_1_id, m3508_location, m3508_speed);
GM6020 gm6020_1(gm6020_1_id, gm6020_location1, gm6020_speed);
GM6020 gm6020_2(gm6020_2_id, gm6020_location2, gm6020_speed);
// Cybergear settings
CybergearController controller(master_id);
CybergearCanInterface interface;
//                              id, dir, limit_speed, limit_current, limit_torque, upper_position_limit, lower_position_limit, calib_direction, position_offset
CybergearSoftwareConfig config1(cyber_ids[0], CCW, 30, 27, 12, 0, -720*M_PI/180, CW, 0);
CybergearSoftwareConfig config2(cyber_ids[1], CCW, 30, 27, 12, 240*M_PI/180, 0, CCW, 0);
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
    float calib_torque = 0; // 5.5
    float calib_speed = 0.03; // rad/s
    Serial.println("\rcalib start");
    // トルク制御モードに変更
    controller.set_run_mode(MODE_CURRENT);
    delay(100);
    controller.send_current_command(cyber_ids, {calib_torque, -calib_torque});
    delay(1000);
    bool flag1 = false;
    bool flag2 = false;
    while(true){ // 速度が一定以下になったら停止
        controller.get_motor_status(motor_status);
        Serial.printf("cyber1(%f), cyber2(%f)\n", motor_status[0].position*45/M_PI, motor_status[1].position*90/M_PI);
        controller.send_current_command(cyber_ids, {calib_torque*sw_configs[0].calib_direction, calib_torque*sw_configs[1].calib_direction});
        if(std::abs(motor_status[0].velocity) < calib_speed && !flag1){
            sw_configs[0].position_offset = -motor_status[0].position;
            Serial.printf("set offset: %f\n", -motor_status[0].position*45/M_PI);
            controller.set_software_config(cyber_ids[0], sw_configs[0]);
            controller.set_run_mode(cyber_ids[0], MODE_POSITION);
            delay(100);
            controller.send_position_command(cyber_ids[0], 0); //-70*M_PI/45);
            Serial.println("motor 0 ok");
            flag1 = true;
        }
        if(std::abs(motor_status[1].velocity) < calib_speed && !flag2){
            sw_configs[1].position_offset = -motor_status[1].position;
            Serial.printf("set offset: %f\n", -motor_status[1].position*90/M_PI);
            controller.set_software_config(cyber_ids[1], sw_configs[1]);
            controller.set_run_mode(cyber_ids[1], MODE_POSITION);
            delay(100);
            controller.send_position_command(cyber_ids[1], 0); //20*M_PI/90);
            Serial.println("motor 1 ok");
            flag2 = true;
        }
        if(flag1 && flag2) {
            break;
        }
        delay(10);
    }
    delay(500);
    controller.send_position_command(cyber_ids[0], 0); //-70*M_PI/45);
    // controller.send_position_command(cyber_ids[0], -70*M_PI/45);
    controller.send_position_command(cyber_ids[1], 0); //20*M_PI/90);
    // controller.send_position_command(cyber_ids[1], 20*M_PI/90);
    controller.get_motor_status(motor_status);
    Serial.printf("cyber1(%f), cyber2(%f)\n", motor_status[0].position*45/M_PI, motor_status[1].position*90/M_PI);
    Serial.println("calib finish");
    return true;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    adc_setup(-10);
    udp.init();
    can_init(RX_PIN, TX_PIN, 1000);
    controller.init(cyber_ids, sw_configs, MODE_POSITION, &interface, 0);
    for(auto id : cyber_ids){
        controller.set_speed_limit(id, 10); // 30
        controller.set_torque_limit(id, 12); //12
        controller.set_current_limit(id, 27); // 27
        controller.set_position_control_gain(id, 30); // 30
        controller.set_velocity_control_gain(id, 1.0, 0.002); // 1.0, 0.002)
        controller.set_current_control_param(id, 0.125, 0.0158, 0.1); // 0.125, 0.0158, 0.1
        add_user_can_func(reconstruct_id(master_id, id, CMD_RESPONSE), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
        // add_user_can_func(reconstruct_id(master_id, id, CMD_RAM_READ), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
        // add_user_can_func(reconstruct_id(master_id, id, CMD_GET_MOTOR_FAIL), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
    }
    // m3508_1.set_angleはloop内でしか使えない。
    m3508_1.setup();
    gm6020_1.setup();
    gm6020_2.setup();
    gm6020_1.set_angle(210);
    gm6020_2.set_angle(260);
    controller.enable_motors();
    calib_motors();
    Serial.println("setup finish");
}

std::vector<float> target_angle = {0, 0, 0, 0, 0};
uint8_t mode;
void loop() {
    controller.get_motor_status(motor_status);
    Serial.printf("@%f, %f, %f, %f, %f\n", motor_status[0].position*45/M_PI, motor_status[1].position*90/M_PI, gm6020_1.get_angle(), gm6020_2.get_angle(), get_adc_deg());
    mode = udp.get_mode();
    if (mode == 0) { // 接続テスト　アクチュエータの位置を初期位置に戻す
        Serial.println(2);
        gm6020_1.set_angle(210);
        gm6020_2.set_angle(260);
        controller.send_position_command(cyber_ids[0], 0);
        controller.send_position_command(cyber_ids[1], 0);    
        m3508_1.set_speed(0);
    }
    else if (mode == 1) { // 位置制御　追従する
        float target = 0;
        Serial.println(3);
        udp.get_data(target_angle); // degree
        // target_angleを表示
        // Serial.printf("target_angle: %f, %f, %f, %f, %f\n", target_angle[0], target_angle[1], target_angle[2], target_angle[3], target_angle[4]);
        // id1 cyber1 第二関節
        target = (target_angle[0] - 160)*4; // 1/4倍減速
        // target = std::min(-720.0f, std::max(0.0f, target));
        controller.send_position_command(cyber_ids[0], target*M_PI/180);
        // id2 cyber2 第一関節
        target = (target_angle[1] + 20)*2; // 1/2倍減速
        // target = std::min(240.0f, std::max(0.0f, target));
        controller.send_position_command(cyber_ids[1], target*M_PI/180);
        // id3 gm1 id3 腕
        target = -target_angle[2] + 30;
        if (target < 0) target += 360;
        gm6020_1.set_angle(target); // 210
        // id4 gm2 id4 手首
        target = target_angle[3] + 80; // 260~180~0~300 tumari 280 ni genten
        if (target < 0) target += 360;
        gm6020_2.set_angle(target);
        // id5 m3508 台座
        target = target_angle[4];
        if (target < 0) target += 360;
        m3508_1.set_angle(target); // 台座 正面が180、反時計回り
    }
    else if (mode == 2) { // サスペンド　その場で停止する
        Serial.println(4);
        m3508_1.set_speed(0);
    }else{
        m3508_1.set_speed(0);
    }
    delay(10);
}