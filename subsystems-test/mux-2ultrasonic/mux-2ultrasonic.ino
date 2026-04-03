#define ECHO_MUX_OUT 34 

// MUX Address Pins
#define A 18
#define B 19
#define C 21

// Separate Trigger Pins
#define TRIG1 5
#define TRIG2 32

void setup() {
  Serial.begin(115200);

  // Initialize MUX Control
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(ECHO_MUX_OUT, INPUT);

  // Initialize Triggers
  pinMode(TRIG1, OUTPUT);
  pinMode(TRIG2, OUTPUT);

  // Ensure everything starts LOW
  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);

  Serial.println("System Booted. Starting Measurements...");
}

float readSensor(int channel, int trigPin) {
  // 1. SELECT MUX CHANNEL
  digitalWrite(A, bitRead(channel, 0)); 
  digitalWrite(B, bitRead(channel, 1));
  digitalWrite(C, bitRead(channel, 2));
  
  // Give the MUX and electrical lines a moment to settle
  delayMicroseconds(100); 

  // 2. TRIGGER SEQUENCE (Mentor's Advice: 5us LOW, 10us HIGH)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. LISTEN (Timeout at 30ms / ~5 meters)
  long duration = pulseIn(ECHO_MUX_OUT, HIGH, 30000);
  
  if (duration == 0) return -1; // No object detected
  return (float)duration / 58.0; // Standard HC-SR04 conversion
}

void loop() {
  // --- SENSOR 1 ---
  float d1 = readSensor(0, TRIG1); // Channel 0, Trig 5
  Serial.print("Sensor 1: ");
  if (d1 < 0) Serial.print("OUT OF RANGE");
  else { Serial.print(d1); Serial.print(" cm"); }
  
  // 4. WAIT 0.5 SECONDS (500ms)
  delay(500);

  // --- SENSOR 2 ---
  float d2 = readSensor(1, TRIG2); // Channel 1, Trig 32
  Serial.print(" | Sensor 2: ");
  if (d2 < 0) Serial.println("OUT OF RANGE");
  else { Serial.print(d2); Serial.println(" cm"); }

  // WAIT 0.5 SECONDS before repeating loop
  delay(500);
}