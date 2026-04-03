#define ECHO_MUX_OUT 34 

// MUX Address Pins
#define MUX_A 18
#define MUX_B 19
#define MUX_C 21

// Individual Trigger Pins for 3 Sensors
const int trigPins[] = {5, 16, 17}; 
const int numSensors = 3;

void setup() {
  Serial.begin(115200);

  // Initialize MUX Control Pins
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  pinMode(ECHO_MUX_OUT, INPUT);

  // Initialize Trigger Pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW); // Ensure they start LOW
  }

  Serial.println("--- 3 Sensor System Ready (Individual Triggers) ---");
}

float getDistance(int index) {
  // 1. SELECT MUX CHANNEL (A=bit0, B=bit1, C=bit2)
  digitalWrite(MUX_A, bitRead(index, 0)); 
  digitalWrite(MUX_B, bitRead(index, 1));
  digitalWrite(MUX_C, bitRead(index, 2));
  
  // Give MUX internal switches time to settle
  delayMicroseconds(50); 

  // 2. TRIGGER SEQUENCE (Mentor's Rule: 5us LOW, then 10us HIGH)
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(5); 
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // 3. LISTEN (Timeout at 30ms = approx 5 meters)
  // pulseIn waits for the pin to go HIGH, then measures until it goes LOW
  long duration = pulseIn(ECHO_MUX_OUT, HIGH, 30000);
  
  if (duration == 0) return -1.0; // Out of range or no connection
  
  // 4. CALCULATION (Standard conversion: microseconds / 58)
  return (float)duration / 58.0;
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    float distance = getDistance(i);
    
    Serial.print("Sensor "); Serial.print(i + 1); Serial.print(": ");
    
    if (distance < 0) {
      Serial.print("FAIL/TIMEOUT");
    } else {
      Serial.print(distance, 1); // Print with 1 decimal place
      Serial.print(" cm");
    }
    
    Serial.print(" | ");

    // 5. THE "SONIC SILENCE" DELAY
    // 0.5 seconds as requested to ensure absolute stability
    delay(500); 
  }
  
  Serial.println(""); // New line after all 3 sensors
}