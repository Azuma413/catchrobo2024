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
    float udp_data[data_size];
    WiFiUDP udp;  // クラス内にudpを含める

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
        udp.begin(port);  // クラスメンバーのudpを使用
        xTaskCreate(udp_read_task, "udp_read_task", 4096, this, 5, nullptr);
    }
    float get_data(int index) {
        return udp_data[index];
    }

private:
    const char* ssid;
    const char* password;

    // タスクのエントリポイント用ラッパー関数
    static void udp_read_task(void* parameter) {
        UDPRead* udpInstance = (UDPRead*)parameter;  // クラスのインスタンスを取得
        while (true) {
            int packet_size = udpInstance->udp.parsePacket();  // クラスメンバーのudpにアクセス
            if (packet_size > 0) {
                if (packet_size == data_size * sizeof(float)) {
                    udpInstance->udp.read((char*)udpInstance->udp_data, data_size * sizeof(float));  // udp_dataへのアクセスをクラスインスタンス経由で
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
