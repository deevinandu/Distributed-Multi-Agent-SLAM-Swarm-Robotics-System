#define ECHO_MUX_OUT 34 

// MUX Address Pins
#define MUX_A 18
#define MUX_B 19
#define MUX_C 21

// Individual Trigger Pins for 4 Sensors
// S1=5, S2=16, S3=17, S4=13
const int trigPins[] = {5, 16, 17, 13}; 
const int numSensors = 4;

void setup() {
  Serial.begin(115200);

  // Initialize MUX Control Pins
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  pinMode(ECHO_MUX_OUT, INPUT);

  // Initialize all 4 Trigger Pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW); 
  }

  Serial.println("--- 4 Sensor System Initialized ---");
}

float getDistance(int index) {
  // 1. SELECT MUX CHANNEL
  // bitRead(0,0)=0, bitRead(1,0)=1, bitRead(2,0)=0, bitRead(3,0)=1 (Pin A)
  // bitRead(0,1)=0, bitRead(1,1)=0, bitRead(2,1)=1, bitRead(3,1)=1 (Pin B)
  digitalWrite(MUX_A, bitRead(index, 0)); 
  digitalWrite(MUX_B, bitRead(index, 1));
  digitalWrite(MUX_C, bitRead(index, 2));
  
  // Settling time for the CD4051B internal switches
  delayMicroseconds(50); 

  // 2. TRIGGER SEQUENCE (Mentor's Rule: 5us LOW, then 10us HIGH)
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(5); 
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // 3. LISTEN (Timeout at 30ms = ~5 meters)
  long duration = pulseIn(ECHO_MUX_OUT, HIGH, 30000);
  
  if (duration == 0) return -1.0; 
  
  // 4. CALCULATION (microseconds / 58 = cm)
  return (float)duration / 58.0;
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    float distance = getDistance(i);
    
    Serial.print("S"); Serial.print(i + 1); Serial.print(": ");
    
    if (distance < 0) {
      Serial.print("FAIL");
    } else {
      Serial.print(distance, 1);
      Serial.print("cm");
    }
    
    Serial.print(" | ");

    // 5. THE "SONIC SILENCE" DELAY
    // 0.5 seconds between each sensor for stability
    delay(500); 
  }
  
  Serial.println(""); // New line after the full scan of 4 sensors
}