#ifndef UDP_READ_WRITE_HPP
#define UDP_READ_WRITE_HPP

#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/task.h"

const int receive_port = 12345; // 受信側のポート番号
const int send_port = 8888; // 送信側のポート番号
const int data_size = 5; // 受信するデータの要素数
const IPAddress receive_ip(192, 168, 0, 77); // ESP32のIPアドレス
const IPAddress send_ip(192, 168, 0, 255); // 送信先のIPアドレス
const IPAddress gateway(192, 168, 0, 1); // ゲートウェイ（ルータ）のIPアドレス
const IPAddress subnet(255, 255, 255, 0); // サブネットマスク

// UDPクラス
class UDPReadWrite {
public:
    UDPReadWrite(const char* ssid, const char* password) : ssid(ssid), password(password) {}

    void init() {
        if (!WiFi.config(receive_ip, gateway, subnet)) {
            Serial.println("Failed to configure");
        }
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.println("Connecting to WiFi...");
        }
        Serial.println("Connected to the WiFi network");
        udp.begin(receive_port);
        xTaskCreate(udp_task, "udp_task", 4096, this, 5, nullptr);
    }
    void get_data(std::vector<float>& data) {
        data.assign(read_data, read_data + data_size);
    }
    void set_data(std::vector<float>& data) {
        if (data.size() == data_size){
            std::copy(data.begin(), data.end(), send_data);
        } else {
            Serial.println("Invalid data size");
        }
    }
    float get_mode() {
        return mode;
    }

private:
    const char* ssid;
    const char* password;
    uint8_t mode = 10;
    float read_data[data_size];
    float send_data[data_size];
    WiFiUDP udp;  // クラス内にudpを含める

    // タスクのエントリポイント用ラッパー関数
    static void udp_task(void* parameter) {
        UDPReadWrite* udpInstance = (UDPReadWrite*)parameter;  // クラスのインスタンスを取得
        while (true) {
            // パケットの受信
            int packet_size = udpInstance->udp.parsePacket();  // クラスメンバーのudpにアクセス
            if (packet_size > 0) {
                if (packet_size == data_size * sizeof(float) + sizeof(uint8_t)) {
                    udpInstance->udp.read(&udpInstance->mode, sizeof(uint8_t));
                    for (int i = 0; i < data_size; i++) {
                        udpInstance->udp.read((char*)&udpInstance->read_data[i], sizeof(float));
                    }
                } else {
                    Serial.println("Invalid packet size");
                    udpInstance->udp.flush();  // クラスメンバーのudpにアクセス
                }
            }
            // パケットの送信
            udpInstance->udp.beginPacket(send_ip, send_port);  // クラスメンバーのudpにアクセス
            udpInstance->udp.write((uint8_t*)&udpInstance->mode, sizeof(uint8_t));
            udpInstance->udp.write((uint8_t*)udpInstance->send_data, data_size * sizeof(float));
            udpInstance->udp.endPacket();  // クラスメンバーのudpにアクセス
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
};

#endif