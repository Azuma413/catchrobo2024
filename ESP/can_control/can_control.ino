#include "DJIMotorCtrlESP.hpp"
#include "adc_read.hpp"

// PIN
// CAN RX 4
// CAN TX 5
// ADC 34

// CAN ID
// 01: CyberGear
// 02: CyberGear
// 03: m3508
// 04: GM6020
// 05: GM6020
const uint8_t RX_PIN = 5;
const uint8_t TX_PIN = 4;

const int cyber_1_id = 1;
const int m3508_1_id = 3;
const int gm6020_1_id = 4;
//                          kp,     ki,     kd,     dead_zone, max_value
pid_param m3508_1_speed(    5,      1,      0.01,   1,      10000);
pid_param m3508_1_location( 0.1,    0.1,    0,      2000,   3000);
pid_param gm6020_1_speed(   10,     0,      0,      1,      16384);
pid_param gm6020_1_location(1.0,    0.1,    0.03,   5,      350);

M3508_P19 m3508_1(m3508_1_id, m3508_1_location, m3508_1_speed);
GM6020 gm6020_1(gm6020_1_id, gm6020_1_location, gm6020_1_speed);
void setup() {
    Serial.begin(115200);
    while(!Serial);
	adc_setup();
    can_init(RX_PIN, TX_PIN, 1000);
    m3508_1.setup();
    gm6020_1.setup();
    // gm6020_1.set_angle(0);
    // m3508_1.set_location(0);
}

float target_angle = 0;
float rate = 0.05;
float location_deg_rate = 2000;
void loop() {
    float degree = get_adc_deg();
    target_angle = target_angle * (1.0 - rate) + degree * rate;
    m3508_1.set_location((int64_t)target_angle*location_deg_rate);
    gm6020_1.set_angle(target_angle);
    Serial.println(target_angle);
    // Serial.print(target_angle);
    // Serial.print(",");
    float angle = gm6020_1.get_angle();
    // Serial.println(angle);
    delay(10);
}