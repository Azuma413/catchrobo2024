#include <Arduino.h>
#include "adc_read.hpp"
#include "udp_read.hpp"

// テスト時の設定
const char* ssid = "NotFreeWiFi";
const char* password = "924865hirekatsu";
// 本番設定
// const char* ssid = "NotFreeSub";
// const char* password = "924865hirekatsu";

// UDP通信設定
UDPRead udp(ssid, password);

void setup()
{
	Serial.begin(115200);
	// adc_setup();
	udp.init();
}

void loop()
{
	// float degree = get_adc_deg();
	// Serial.println(degree);
	uint8_t mode = udp.get_mode();
	std::vector<float> data;
	udp.get_data(data);
	Serial.printf("mode: %d\n", mode);
	Serial.printf("data: %f, %f, %f, %f, %f\n", data[0], data[1], data[2], data[3], data[4]);
	delay(100);
}