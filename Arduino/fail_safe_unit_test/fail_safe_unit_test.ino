// Ultrasonic Sensor Pins (3-wire mode)
#define uss_left 12
#define uss_mid 10
#define uss_right 4
#define threshold 15

void setup(){
  Serial.begin(9600); 
}

void loop(){
  // Check if any of the distances from the 3 ultrasonic sensors were below the threshold.
  if (getUltrasonicDistance(uss_mid) < threshold ||
      getUltrasonicDistance(uss_left) < threshold ||
      getUltrasonicDistance(uss_right) < threshold){
    Serial.println("Stopped");
    delay(5000);
    return;
  }
}

// Calculate distance from ultrasonic sensor readings
int getUltrasonicDistance(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(50);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  int result = pulseIn(pin, HIGH, (unsigned long)60000) / 29 / 2;
  // Serial.println(result);
  return result;
}
