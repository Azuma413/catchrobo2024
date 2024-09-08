#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/task.h"

const int port = 9999;
const int data_size = 5;

const IPAddress ip(192, 168, 0, 77);
const IPAddress gateway(192, 168, 0, 1);
const IPAddress subnet(255, 255, 255, 0);

// UDPクラス
class UDPRead {
public:
    UDPRead(const char* ssid, const char* password) : ssid(ssid), password(password) {}

    void init() {
        if (!WiFi.config(ip, gateway, subnet)) {
            Serial.println("Failed to configure");
        }
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.println("Connecting to WiFi...");
        }
        Serial.println("Connected to the WiFi network");
        udp.begin(port);
        xTaskCreate(udp_read_task, "udp_read_task", 4096, this, 5, nullptr);
    }
    void get_data(std::vector<float>& data) {
        data.assign(udp_data, udp_data + data_size);
    }

    float get_mode() {
        return mode;
    }

private:
    const char* ssid;
    const char* password;
    uint8_t mode = 10;
    float udp_data[data_size];
    WiFiUDP udp;  // クラス内にudpを含める

    // タスクのエントリポイント用ラッパー関数
    static void udp_read_task(void* parameter) {
        UDPRead* udpInstance = (UDPRead*)parameter;  // クラスのインスタンスを取得
        while (true) {
            int packet_size = udpInstance->udp.parsePacket();  // クラスメンバーのudpにアクセス
            if (packet_size > 0) {
                if (packet_size == data_size * sizeof(float) + sizeof(uint8_t)) {
                    udpInstance->udp.read(&udpInstance->mode, sizeof(uint8_t));
                    for (int i = 0; i < data_size; i++) {
                        udpInstance->udp.read((char*)&udpInstance->udp_data[i], sizeof(float));
                    }
                } else {
                    Serial.println("Invalid packet size");
                    udpInstance->udp.flush();  // クラスメンバーのudpにアクセス
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
};
