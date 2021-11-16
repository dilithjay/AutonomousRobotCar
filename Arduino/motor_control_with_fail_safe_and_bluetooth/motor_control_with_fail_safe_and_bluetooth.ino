// L298N motor driver control pins
const byte enA = 11;
const byte in1 = 9;
const byte in2 = 8;
const byte in3 = 7;
const byte in4 = 6;
const byte enB = 5;

// Ultrasonic Sensor Pins (3-wire mode)
const byte uss_left = 12;
const byte uss_mid = 10;
const byte uss_right = 4;

byte left_speed, right_speed;
bool speed_received = false;

// Minimum distance for collision in centimeters
const short threshold = 15;

void setup() {
  // Define pin modes
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize Serial for printing results
  Serial.begin(9600);
  // Ensure Serial is ready
  while (! Serial);
}

void loop() {
  // Check if any of the distances from the 3 ultrasonic sensors were below the threshold.
  /*if (getUltrasonicDistance(uss_mid) < threshold ||
      getUltrasonicDistance(uss_left) < threshold ||
      getUltrasonicDistance(uss_right) < threshold){
    stop_and_wait(5000);
    return;
  }*/
  

  if (Serial.available() == 2){
    left_speed = Serial.read();
    right_speed = Serial.read();
    Serial.print("L: ");
    Serial.println(left_speed);
    Serial.print("R: ");
    Serial.println(right_speed);
  }
  setLeftSpeed(left_speed);
  setRightSpeed(right_speed);
}

void stop_and_wait(int millisecs){
    setLeftSpeed(0);
    setRightSpeed(0);
    delay(millisecs);  
}

void setLeftSpeed(byte num){
  analogWrite(enB, num);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void setRightSpeed(byte num){
  analogWrite(enA, num);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// Calculate distance from ultrasonic sensor readings
int getUltrasonicDistance(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(20);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  int result = pulseIn(pin, HIGH, (unsigned long)60000) / 29 / 2;
  // Serial.println(result);
  return result;
}
