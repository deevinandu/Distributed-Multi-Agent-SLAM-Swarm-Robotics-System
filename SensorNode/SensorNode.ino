/*
 * SensorNode.ino - Distributed Sensor Agent (Broadcast Version)
 */
#include <esp_now.h>
#include <WiFi.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SOUND_SPEED 0.0343

#define WIFI_SSID "Gia"
#define WIFI_PASSWORD "e7jt92zp"

typedef struct struct_message {
    float distance;
} struct_message;
struct_message myData;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Set to Station mode and connect to sync Channel with Main Agent
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("[WIFI] Syncing Channel with Gia...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Channel Synced!");

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void loop() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms = ~5 meters
    
    // Distance = Time * Speed / 2 (result in Centimeters)
    float distCm = (duration * SOUND_SPEED / 2.0);
    
    // If timeout (duration 0), set to 400cm
    if (duration == 0) distCm = 400.0;

    myData.distance = distCm; // We are now sending CM
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    // Detailed Debugging for the user
    Serial.printf("[V2V] Raw Duration: %ld us | Distance: %.2f cm\n", duration, distCm);
    
    delay(50); // 20Hz Update
}
