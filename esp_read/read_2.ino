#include <Arduino.h>
/*
	ESP-IDF ADコンバータAPI キャリブレーション利用
	電圧測定
*/

// ADCドライバ
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define DEFAULT_VREF 1100 // eFuse Vrefがチップに記録されていない場合に利用される（らしい）

// 校正計算用
adc_unit_t unit1 = ADC_UNIT_1;			   //	ADC_UNIT_1, ADC_UNIT_2, ADC_UNIT_BOTH, ADC_UNIT_ALTER (1-3,7) + ADC_UNIT_MAX
// adc_unit_t unit2 = ADC_UNIT_2;			   //	ADC_UNIT_1, ADC_UNIT_2, ADC_UNIT_BOTH, ADC_UNIT_ALTER (1-3,7) + ADC_UNIT_MAX
adc_atten_t atten = ADC_ATTEN_DB_11;		   //	ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6, ADC_ATTEN_DB_11 (0-3) + ADC_ATTEN_DB_MAX
adc_bits_width_t width_bit = ADC_WIDTH_BIT_12; //	ADC_WIDTH_BIT_9, ADC_WIDTH_BIT_10, DC_WIDTH_BIT_11, ADC_WIDTH_BIT_12 (0-3) + ADC_WIDTH_MAX
//adc1_channel_t channel = ADC1_CHANNEL_4;	   //	GPIO32 if ADC1
adc1_channel_t channel_1 = ADC1_GPIO32_CHANNEL; //	ADC1_GPIO32_CHANNEL マクロがあるので表を調べなくても大丈夫っぽい
// adc2_channel_t channel_2 = ADC2_GPIO27_CHANNEL; //	ADC2_GPIO27_CHANNEL

// ADC補正用構造体へのポインタ
esp_adc_cal_characteristics_t *adc_chars_1;
// esp_adc_cal_characteristics_t *adc_chars_2;

uint8_t adc1_pin = 34;
// uint8_t adc2_pin = 27;
float_t R1 = 330; //	330Ω 金属皮膜抵抗 328
float_t R2 = 220; //	220Ω 金属皮膜抵抗 217

static void check_efuse()
{
	//Check TP is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
	{
		printf("eFuse Two Point: Supported\n");
	}
	else
	{
		printf("eFuse Two Point: NOT supported\n");
	}

	//Check Vref is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
	{
		printf("eFuse Vref: Supported\n");
	}
	else
	{
		printf("eFuse Vref: NOT supported\n");
	}
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
	if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
	{
		printf("Characterized using Two Point Value\n");
	}
	else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
	{
		printf("Characterized using eFuse Vref\n");
	}
	else
	{
		printf("Characterized using Default Vref\n");
	}
}

static void print_calibration_status()
{
	// 構造体を一応出力
	Serial.println("-------------Calibration STATUS-------------");
	Serial.println("---------ADC1---------");
	Serial.print("ADC num : ");
	Serial.println(adc_chars_1->adc_num);
	Serial.print("Attenuation : ");
	Serial.println(adc_chars_1->atten);
	Serial.print("Bit width : ");
	Serial.println(adc_chars_1->bit_width);
	Serial.print("Gradient of ADC-Voltage curve : ");
	Serial.println(adc_chars_1->coeff_a);
	Serial.print("Offset of ADC-Voltage curve : ");
	Serial.println(adc_chars_1->coeff_b);
	Serial.print("vref table : ");
	Serial.println(adc_chars_1->vref);
	// Serial.println("---------ADC2---------");
	// Serial.print("ADC num : ");
	// Serial.println(adc_chars_2->adc_num);
	// Serial.print("Attenuation : ");
	// Serial.println(adc_chars_2->atten);
	// Serial.print("Bit width : ");
	// Serial.println(adc_chars_2->bit_width);
	// Serial.print("Gradient of ADC-Voltage curve : ");
	// Serial.println(adc_chars_2->coeff_a);
	// Serial.print("Offset of ADC-Voltage curve : ");
	// Serial.println(adc_chars_2->coeff_b);
	// Serial.print("vref table : ");
	// Serial.println(adc_chars_2->vref);
	Serial.println("-------------end Calibration STATUS-------------");
}

void setup()
{
	// put your setup code here, to run once:
	// シリアル通信を開始
	Serial.begin(115200);

	// 一応GPIOピンのモードを変えておく
	pinMode(adc1_pin, ANALOG);
	// pinMode(adc2_pin, ANALOG);

	// チップサポートを確認
	check_efuse();

	//Configure ADC
	//	ADC1
	// ADC1は解像度も保持できる
	adc1_config_width(width_bit);
	adc1_config_channel_atten(channel_1, atten);
	//	ADC2
	// ADC2は解像度を保持できないらしい・・・？のでアッテネータの設定のみ
	// adc2_config_channel_atten((adc2_channel_t)channel_2, atten);

	// ADC校正構造体のサイズを取得
	//	voidからのポインタキャストとかめんどくさいンゴねぇ
	adc_chars_1 = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t)); //	ADC1
	// adc_chars_2 = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t)); //	ADC2

	// 校正APIに設定値と構造体を渡す＋ついでにキャリブレーションタイプ取得
	//	ADC1
	esp_adc_cal_value_t val_type_1 = esp_adc_cal_characterize(
		unit1,		  // adc_unit_t ADCユニット番号
		atten,		  // adc_atten_t アッテネータ設定値
		width_bit,	  // adc_bits_width_t ADC解像度
		DEFAULT_VREF, // uint32_t デフォルト校正値
		adc_chars_1); // esp_adc_cal_characteristics_t * キャリブレーション用の構造体へのポインタ

	//	ADC2
	// esp_adc_cal_value_t val_type_2 = esp_adc_cal_characterize(
	// 	unit2,		  // adc_unit_t ADCユニット番号
	// 	atten,		  // adc_atten_t アッテネータ設定値
	// 	width_bit,	  // adc_bits_width_t ADC解像度
	// 	DEFAULT_VREF, // uint32_t デフォルト校正値
	// 	adc_chars_2); // esp_adc_cal_characteristics_t * キャリブレーション用の構造体へのポインタ

	// ADCキャリブレーションタイプを確認
	print_char_val_type(val_type_1);
	// print_char_val_type(val_type_2);

	// adc2の場合他のadc1のGPIOピンを経由してキャリブレーションできるらしい・・・？
	//	adc2_vref_to_gpio(---GPIONUM---);
}

void loop()
{
	// APIからの取得 ADC1
	uint32_t raw_1 = adc1_get_raw((adc1_channel_t)channel_1);
	// ADC2はちょっと面倒・・・
	// int raw_2;
	// adc2_get_raw((adc2_channel_t)channel_2, adc_chars_2->bit_width, &raw_2);

	// キャリブレーションの補正値が出てしまうので、raw_1が0なら計算しない
	if (raw_1 > 0)
	{
		Serial.println("---- ADC1 ----");
		Serial.print("Raw Data : ");
		Serial.println(raw_1);

		uint32_t voltage = esp_adc_cal_raw_to_voltage(raw_1, adc_chars_1); // return mV
		Serial.print("Raw To Voltage : ");
		Serial.println(voltage);

		Serial.print("Restored Voltage : ");
		Serial.printf("%4.3f", float_t((voltage / float_t(R2 / (R1 + R2))) / 1000)); // mVなので単位をVに直す
		Serial.println(" V");
	}
	else
	{
		Serial.println("-- ADC1 is No Signal. --");
	}

	// if (raw_2 > 0)
	// {
	// 	Serial.println("---- ADC2 ----");
	// 	Serial.print("Raw Data : ");
	// 	Serial.println(raw_2);

	// 	uint32_t voltage = esp_adc_cal_raw_to_voltage(raw_2, adc_chars_1); // return mV
	// 	Serial.print("Raw To Voltage : ");
	// 	Serial.println(voltage);

	// 	Serial.print("Restored Voltage : ");
	// 	Serial.printf("%4.3f", float_t((voltage / float_t(R2 / (R1 + R2))) / 1000)); // mVなので単位をVに直す
	// 	Serial.println(" V");
	// }
	// else
	// {
	// 	Serial.println("-- ADC2 is No Signal. --");
	// }

	// キャリブレーション等を変更できる様に
	if (Serial.available())
	{
		int inputchar = Serial.read();

		// 面倒なのでアッテネーターを変更できる様に
		if (char(inputchar) == '1')
		{
			adc1_config_channel_atten(channel_1, ADC_ATTEN_DB_0);
			Serial.println("Attenuation mode : 0 db");
		}
		else if (char(inputchar) == '2')
		{
			adc1_config_channel_atten(channel_1, ADC_ATTEN_DB_2_5);
			Serial.println("Attenuation mode : 2.5db");
		}
		else if (char(inputchar) == '3')
		{
			adc1_config_channel_atten(channel_1, ADC_ATTEN_DB_6);
			Serial.println("Attenuation mode : 6db");
		}
		else if (char(inputchar) == '4')
		{
			adc1_config_channel_atten(channel_1, ADC_ATTEN_DB_11);
			Serial.println("Attenuation mode : 11db");
		}
		else if (char(inputchar) == 'A')
		{
			// coeff_a 補正値 + 100
			adc_chars_1->coeff_a = adc_chars_1->coeff_a + 100;
			Serial.print("coeff_a +100 to :");
			Serial.println(adc_chars_1->coeff_a);
		}
		else if (char(inputchar) == 'S')
		{
			// coeff_a 補正値 - 100
			adc_chars_1->coeff_a = adc_chars_1->coeff_a - 100;
			Serial.print("coeff_a -100 to :");
			Serial.println(adc_chars_1->coeff_a);
		}
		else if (char(inputchar) == 'D')
		{
			// coeff_b 補正値 + 5
			adc_chars_1->coeff_b = adc_chars_1->coeff_b + 5;
			Serial.print("coeff_b +5 to :");
			Serial.println(adc_chars_1->coeff_b);
		}
		else if (char(inputchar) == 'F')
		{
			// coeff_b 補正値 - 5
			adc_chars_1->coeff_b = adc_chars_1->coeff_b - 5;
			Serial.print("coeff_b -5 to :");
			Serial.println(adc_chars_1->coeff_b);
		}
		else if (char(inputchar) == 'W')
		{
			// 解像度トグル
			uint8_t _current_val = adc_chars_1->bit_width;
			if (_current_val > 2)
			{
				adc_chars_1->bit_width = (adc_bits_width_t)0;
			}
			else
			{
				adc_chars_1->bit_width = (adc_bits_width_t)(_current_val + 1);
			}
			Serial.print("Bit width change to :");
			Serial.println(adc_chars_1->bit_width);
		}
		else if (char(inputchar) == 'Z')
		{
			// デフォルトリセット
			esp_adc_cal_characterize(unit1, atten, width_bit, DEFAULT_VREF, adc_chars_1);
			esp_adc_cal_characterize(unit2, atten, width_bit, DEFAULT_VREF, adc_chars_2);
			Serial.println("-------------RESET Calibration-------------");
			// 設定値再表示
			print_calibration_status();
		}
		else if (char(inputchar) == 'P')
		{
			// 状態表示
			print_calibration_status();
		}
	}

	delay(1000);
}