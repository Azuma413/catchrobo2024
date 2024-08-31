#include "driver/twai.h"
#include <Arduino.h>

const uint8_t RX_PIN = 27;
const uint8_t TX_PIN = 26;

void setup(){
    Serial.begin(115200);
    while(!Serial);
    twai_status_info_t now_twai_status;
    auto status = twai_get_status_info(&now_twai_status);
    if(status != ESP_ERR_INVALID_STATE){
        Serial.println("can is installed do not need install again");
        return;
    }
    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(TX_PIN), gpio_num_t(RX_PIN), TWAI_MODE_NO_ACK);
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

void loop(){
    int set_current = 1000;
    twai_message_t tx_msg;
    tx_msg.data_length_code=8;
    tx_msg.identifier = 0x1FE;
    tx_msg.self=0;
    tx_msg.extd=0;
    tx_msg.data[0] = 0;
    tx_msg.data[1] = 0;
    tx_msg.data[2] = 0;
    tx_msg.data[3] = 0;
    tx_msg.data[4] = 0;
    tx_msg.data[5] = 0;
    tx_msg.data[6] = set_current >> 8;
    tx_msg.data[7] = set_current&0xff;
    twai_transmit(&tx_msg,portMAX_DELAY);
    delay(1000);
}







// #include "DJIMotorCtrlESP.hpp"

// // PIN
// // CAN RX 26
// // CAN TX 27
// // ADC 34

// // CAN ID
// // 01: m3508
// // 02: CyberGear
// // 03: CyberGear
// // 04: GM6020
// // 05: GM6020
// const uint8_t RX_PIN = 27;
// const uint8_t TX_PIN = 26;
// GM6020 gm6020_1(4);

// void setup() {
//     Serial.begin(115200);
//     while(!Serial);
//     can_init(RX_PIN, TX_PIN, 100);
//     gm6020_1.setup();
//     gm6020_1.set_angle(180.0);
// }

// void loop() {
//     float current = gm6020_1.get_curunt_ma();
//     Serial.println(current);
//     float angle = gm6020_1.get_angle();
//     Serial.println(angle);
//     delay(100);
// }