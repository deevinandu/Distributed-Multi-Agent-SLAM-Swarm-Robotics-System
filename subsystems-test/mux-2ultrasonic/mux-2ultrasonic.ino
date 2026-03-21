#define TRIG 5
#define ECHO_MUX_OUT 34 // Common Pin on MUX

#define A 18
#define B 19
#define C 21

void setup() {
  Serial.begin(115200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO_MUX_OUT, INPUT);

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
}

float getDistance(int channel) {
  // Select MUX Channel
  // A is least significant bit (bit 0), B is bit 1, C is bit 2
  digitalWrite(A, bitRead(channel, 0)); 
  digitalWrite(B, bitRead(channel, 1));
  digitalWrite(C, bitRead(channel, 2));

  delayMicroseconds(10); // Wait for MUX to stabilize

  // Trigger the sensors
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read only the echo from the selected channel
  long duration = pulseIn(ECHO_MUX_OUT, HIGH, 30000);
  return duration * 0.034 / 2;
}

void loop() {
  float dist1 = getDistance(0); // Channel Y0
  delay(60);                    // Wait for sound waves to clear
  
  float dist2 = getDistance(1); // Channel Y1
  delay(60);

  Serial.print("Sensor 1: ");
  Serial.print(dist1);
  Serial.print(" cm | Sensor 2: ");
  Serial.print(dist2);
  Serial.println(" cm");

  delay(200); // Frequency of updates
}