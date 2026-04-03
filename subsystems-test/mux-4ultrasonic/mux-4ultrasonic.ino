#define ECHO_MUX_OUT 34 

// MUX Address Pins (Only 2 lines needed for 4 channels)
#define MUX_A 18
#define MUX_B 19

// Individual Trigger Pins for 4 Sensors
const int trigPins[] = {5, 16, 17, 13}; 
const int numSensors = 4;

void setup() {
  Serial.begin(115200);

  // Initialize MUX Control Pins
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(ECHO_MUX_OUT, INPUT);

  // Initialize all 4 Trigger Pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW); 
  }

  Serial.println("--- 4 Sensors / 2 MUX Lines System Ready ---");
}

float getDistance(int index) {
  // 1. SELECT MUX CHANNEL (Using only A and B)
  digitalWrite(MUX_A, bitRead(index, 0)); // Selects 0 or 1
  digitalWrite(MUX_B, bitRead(index, 1)); // Selects 0 or 1
  
  // Internal switch settling time
  delayMicroseconds(50); 

  // 2. TRIGGER SEQUENCE (5us LOW, then 10us HIGH)
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(5); 
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // 3. LISTEN (30ms timeout)
  long duration = pulseIn(ECHO_MUX_OUT, HIGH, 30000);
  
  if (duration == 0) return -1.0; 
  
  // 4. CONVERSION (Standard uS / 58)
  return (float)duration / 58.0;
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    float distance = getDistance(i);
    
    Serial.print("S"); Serial.print(i + 1); Serial.print(": ");
    
    if (distance < 0) {
      Serial.print("TIMEOUT");
    } else {
      Serial.print(distance, 1);
      Serial.print("cm");
    }
    
    Serial.print(" | ");

    // 0.5s stability delay
    delay(500); 
  }
  
  Serial.println(""); 
}