// L298N motor driver control pins
#define enA 9
#define in1 8
#define in2 7
#define in3 5
#define in4 4
#define enB 3

// Ultrasonic Sensor Pins (3-wire mode)
#define uss_left 12
#define uss_mid 11
#define uss_right 2

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
    setLeftSpeed(left_speed);
    right_speed = Serial.read();
    setRightSpeed(right_speed);
  }
}

void stop_and_wait(int millisecs){
    setLeftSpeed(0);
    setRightSpeed(0);
    delay(millisecs);  
}

void setLeftSpeed(byte num){
  Serial.print("Left Speed: ");
  Serial.println(num);
  analogWrite(enB, num);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void setRightSpeed(byte num){
  Serial.print("Right Speed: ");
  Serial.println(num);
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
