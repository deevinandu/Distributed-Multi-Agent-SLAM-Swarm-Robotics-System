/*
 * SensorNode.ino - Distributed Sensor Agent (Broadcast Version)
 */
#include <esp_now.h>
#include <WiFi.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SOUND_SPEED 0.0343

typedef struct struct_message {
    float distance;
} struct_message;
struct_message myData;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  // Set to 0 to find channel from WiFi
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void loop() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    float dist = (duration * SOUND_SPEED / 2.0) / 100.0;
    if (duration == 0) dist = 4.0;

    myData.distance = dist;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    Serial.printf("[V2V] Sending: %.2fm\n", dist);
    delay(40); // 25Hz Scan Speed
}
