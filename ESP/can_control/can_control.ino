#include "DJIMotorCtrlESP.hpp"
#include "adc_read.hpp"
#include "cybergear_controller.hh"
#include "cybergear_can_interface.hpp"
// #include "cybergear_driver.hh"

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
pid_param m3508_location( 0.1,    0.1,    0,      2000,   3000);
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

int calc_identifier(uint8_t master_id, uint8_t motor_id){
    unsigned long identifier = (master_id << 8) | motor_id;
    identifier |= 0x12000000;
    Serial.println(identifier);
    return identifier;
}

bool calib_motors(){
    float calib_torque = 0.5;
    float calib_speed = 0.1; // rad/s
    // トルク制御モードに変更
    controller.set_run_mode(MODE_CURRENT);
    // トルク0.5くらいで回転
    controller.send_current_command(cyber_ids, {calib_torque*sw_configs[0].calib_direction, calib_torque*sw_configs[1].calib_direction});
    delay(1000);
    bool flag = false;
    while(true){ // 速度が一定以下になったら停止
        controller.get_motor_status(motor_status);
        if(motor_status[0].velocity < calib_speed){
            controller.set_mech_position_to_zero(cyber_ids[0]);
            controller.send_current_command(cyber_ids[0], 0);
            if(!flag) flag = true;
            else break;
        }
        if(motor_status[1].velocity < calib_speed){
            controller.set_mech_position_to_zero(cyber_ids[1]);
            controller.send_current_command(cyber_ids[1], 0);
            if(!flag) flag = true;
            else break;
        }
    }
    // 位置制御モードに変更
    controller.set_run_mode(MODE_POSITION);
    return true;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    adc_setup();
    can_init(RX_PIN, TX_PIN, 1000);
    controller.init(cyber_ids, sw_configs, MODE_POSITION, &interface, 0);
    for(auto id : cyber_ids){
        controller.set_position_control_gain(id, 30);
        controller.set_velocity_control_gain(id, 1.0, 0.002);
        controller.set_current_control_param(id, 0.125, 0.0158, 0.1);
        add_user_can_func(calc_identifier(master_id, id), std::bind(&CybergearCanInterface::on_receive, &interface, std::placeholders::_1));
    }
    m3508_1.setup();
    gm6020_1.setup();
    gm6020_2.setup();
    controller.enable_motors();
    calib_motors();
}

float target_angle = 0;
float rate = 0.05;
float location_deg_rate = 2000;
void loop() {
    float degree = get_adc_deg();
    target_angle = target_angle * (1.0 - rate) + degree * rate;
    m3508_1.set_location((int64_t)target_angle*location_deg_rate);
    gm6020_1.set_angle(target_angle);
    // Serial.print(target_angle);
    // Serial.print(",");
    // float angle = gm6020_1.get_angle();
    // Serial.println(angle);
    target_angle = target_angle*M_PI/180;
    controller.send_position_command(cyber_ids, {target_angle, target_angle});
    controller.get_motor_status(motor_status);
    Serial.println(motor_status[0].position);
    delay(10);
}