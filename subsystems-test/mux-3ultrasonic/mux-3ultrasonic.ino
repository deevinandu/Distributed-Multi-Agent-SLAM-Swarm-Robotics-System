#define ECHO_MUX_OUT 34 
#define A 18
#define B 19
#define C 21

// TRIG PINS: Sensor 1=5, Sensor 2=32, Sensor 3=33, Sensor 4=25
int trigPins[] = {5, 32, 33, 25}; 

void setup() {
  Serial.begin(115200);
  
  pinMode(ECHO_MUX_OUT, INPUT);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);

  // Initialize all trigger pins as outputs and set them LOW
  for(int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
  }

  Serial.println("System Initialized - Individual Triggers");
}

float getDistance(int sensorIndex) {
  // 1. Select the correct MUX channel for the ECHO
  digitalWrite(A, bitRead(sensorIndex, 0)); 
  digitalWrite(B, bitRead(sensorIndex, 1));
  digitalWrite(C, bitRead(sensorIndex, 2));
  delay(10); // Let the MUX switch settle

  // 2. Trigger ONLY the specific sensor
  digitalWrite(trigPins[sensorIndex], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[sensorIndex], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[sensorIndex], LOW);

  // 3. Measure the echo pulse
  // Timeout set to 30000us (~5 meters)
  long duration = pulseIn(ECHO_MUX_OUT, HIGH, 30000);
  
  // 4. Filter out immediate noise (anything less than 2cm)
  if (duration < 110 || duration == 0) return -1; 
  
  return duration * 0.034 / 2;
}

void loop() {
  for (int i = 0; i < 4; i++) {
    float d = getDistance(i);
    
    Serial.print("S"); Serial.print(i + 1); Serial.print(": ");
    if (d < 0) {
      Serial.print("FAIL  ");
    } else {
      Serial.print(d, 1); // 1 decimal place
      Serial.print("cm ");
    }
    Serial.print("| ");

    // 5. CRITICAL: Wait for the sound to vanish before moving to the next sensor
    delay(80); 
  }
  
  Serial.println(""); // New line after all 4 sensors
  delay(200); // Wait a bit before the next full scan
}