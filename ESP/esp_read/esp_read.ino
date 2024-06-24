#include <Arduino.h>
#include "adc_read.hpp"
#include "udp_read.hpp"

float udp_data[4];

void setup()
{
	Serial.begin(115200);
	adc_setup();
	wifi_setup();
}

void loop()
{
	float degree = get_adc_deg();
	udp_read(udp_data);
	Serial.println(udp_data[0]);
	delay(5);
}