#ifndef ADC_READ_HPP
#define ADC_READ_HPP

#include <driver/adc.h>
#include <esp_adc_cal.h>

#define DEFAULT_VREF 1100

adc_unit_t unit1 = ADC_UNIT_1;
adc_atten_t atten = ADC_ATTEN_DB_11;
adc_bits_width_t width_bit = ADC_WIDTH_BIT_12;
adc1_channel_t channel_1 = ADC1_CHANNEL_6;
esp_adc_cal_characteristics_t *adc_chars_1;
uint8_t adc1_pin = 34;
std::vector<uint32_t> raw_data;
int window_size = 8;
float offset = 0;

void adc_setup(float offset_){
	offset = offset_;
    pinMode(adc1_pin, ANALOG);
	adc1_config_width(width_bit);
	adc1_config_channel_atten(channel_1, atten);
	adc_chars_1 = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t)); //	ADC1
	esp_adc_cal_value_t val_type_1 = esp_adc_cal_characterize(
		unit1,		  // adc_unit_t ADCユニット番号
		atten,		  // adc_atten_t アッテネータ設定値
		width_bit,	  // adc_bits_width_t ADC解像度
		DEFAULT_VREF, // uint32_t デフォルト校正値
		adc_chars_1); // esp_adc_cal_characteristics_t * キャリブレーション用の構造体へのポインタ
}

float vol2deg(uint32_t vol){
	return (float)(vol-142)/(3176.0 - 142.0)*308.5714;//360.0;
}

float get_adc_deg(){
    uint32_t raw_1 = adc1_get_raw((adc1_channel_t)channel_1);
	if (raw_data.size() < window_size){
		raw_data.push_back(raw_1);
	}else{
		raw_data.erase(raw_data.begin());
		raw_data.push_back(raw_1);
	}
	uint32_t sum = 0;
	for (int i = 0; i < raw_data.size(); i++){
		sum += raw_data[i];
	}
	raw_1 = sum / raw_data.size();
    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw_1, adc_chars_1);
    return vol2deg(voltage) - offset;
}

#endif
