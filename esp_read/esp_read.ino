#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <stdio.h>

#define SerialPort Serial
#define LED_PIN 2
#define AD_PIN 34

esp_adc_cal_characteristics_t adcChar;

float vol2deg(int vol){
  return (float)vol/2270.0 * 180;
}

void setup() 
{
  esp_adc_cal_chracteristics(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adcChar);
  pinMode(LED_PIN, OUTPUT);
  SerialPort.begin(115200);
  analogSetAttenuation(ADC_11db);
  pinMode(AD_PIN, ANALOG);
  digitalWrite(LED_PIN, HIGH);
}

void loop() 
{
  SerialPort.println(String(analogReadMilliVolts(AD_PIN)));
  delay(100);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}