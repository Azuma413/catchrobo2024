#include <WiFi.h>
#include <WiFiUdp.h>

// テスト時の設定
const char* ssid = "NotFreeWiFi";
const char* password = "924865hirekatsu";
// 本番設定
// const char* ssid = "NotFreeSub";
// const char* password = "924865hirekatsu";

int port = 9999;
int data_size = 4;

const IPAddress ip(192, 168, 0, 77);
const IPAddress subnet(255, 255, 255, 0);
WiFiUDP udp;

void wifi_setup(){
    if (!WiFi.config(ip, ip, subnet)) {
        Serial.println("Failed to configure");
    }
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
    udp.begin(port);
}

// Float32[4]のデータを受信する
void udp_read(float* data){
    int packet_size = udp.parsePacket();
    if (packet_size > 0){
        if (packet_size == data_size * sizeof(float)){
            udp.read((char*)data, data_size * sizeof(float));
        }else{
            Serial.println("Invalid packet size");
            udp.flush();
        }
    }else{
        //Serial.println("No packet");
    }
}