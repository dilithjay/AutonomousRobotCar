#include <AutoPID.h>
#include "TimerOne.h"

// L298N motor driver control pins
const byte enA = 11;
const byte in1 = 9;
const byte in2 = 8;
const byte in3 = 7;
const byte in4 = 6;
const byte enB = 5;

// Variables for speed encoders
const byte encA = 3;
const byte encB = 2;
const float disk_slots = 20.0f;
const float wheel_diameter = 66.0f;
const float interval_secs = .1f;
volatile int counter_A = 0;
volatile int counter_B = 0;
float rpm_A = 0.0f, rpm_B = 0.0f;

// Variables for byte to rpm mapping
const float min_byte = 70, max_byte = 255;
const float max_rpm = 700, min_rpm = 300;

byte left_speed, right_speed;
bool speed_received = false;

// Variables for PID control
const double KP = 2, KI = 10, KD = 1;
double input_A, output_A, target_A = 0;
double input_B, output_B, target_B = 0;
AutoPID pid_A(&input_A, &target_A, &output_A, 0, 255, KP, KI, KD);
AutoPID pid_B(&input_B, &target_B, &output_B, 0, 255, KP, KI, KD);

// Ultrasonic Sensor Pins (3-wire mode)
const byte uss_left = 12;
const byte uss_mid = 10;
const byte uss_right = 4;

// Minimum distance for collision in centimeters
const byte threshold = 15;

void ISR_count_A() {
  counter_A++;
}

void ISR_count_B() {
  counter_B++;
}

void ISR_timerone()
{
  Timer1.detachInterrupt();
  rpm_A = (counter_A / disk_slots) / interval_secs * 60.0f;
  counter_A = 0;
  rpm_B = (counter_B / disk_slots) / interval_secs * 60.0f;
  counter_B = 0;
  //print_RPMs();
  input_A = (double) rpm_A;
  input_B = (double) rpm_B;
  print_PID_values();
  Timer1.attachInterrupt( ISR_timerone );
}

void setup() {
  // Define pin modes
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize Serial for printing results
  Serial.begin(9600);
  // Ensure Serial is ready
  while (! Serial);

  Timer1.initialize(interval_secs * 1000000.0f); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt (encA), ISR_count_A, RISING);
  attachInterrupt(digitalPinToInterrupt (encB), ISR_count_B, RISING);
  Timer1.attachInterrupt( ISR_timerone );

  pid_A.setTimeStep(100);
  pid_B.setTimeStep(100);
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
    target_A = get_rpm_from_byte(left_speed);
    target_B = get_rpm_from_byte(right_speed);
    Serial.println("-----------Received-----------");
    print_speeds();
  }
  
  pid_A.run();
  pid_B.run();
  
  left_speed = (byte) output_A;
  right_speed = (byte) output_B;
  
  setLeftSpeed(left_speed);
  setRightSpeed(right_speed);
  
  delay(100);
}

double get_rpm_from_byte(byte num){
  return (num - min_byte) / (max_byte - min_byte) * (max_rpm - min_rpm) + min_rpm;
}

void stop_and_wait(int millisecs) {
  setLeftSpeed(0);
  setRightSpeed(0);
  delay(millisecs);
}

void setLeftSpeed(byte num) {
  analogWrite(enB, num);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void setRightSpeed(byte num) {
  analogWrite(enA, num);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// Calculate distance from ultrasonic sensor readings
int getUltrasonicDistance(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(20);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  int result = pulseIn(pin, HIGH, (unsigned long)60000) / 29 / 2;
  // Serial.println(result);
  return result;
}

void print_RPMs() {
  Serial.print("Motor Speed 1: ");
  Serial.print(rpm_A);
  Serial.print(" RPM - ");
  Serial.print("Motor Speed 2: ");
  Serial.print(rpm_B);
  Serial.println(" RPM");
}

void print_PID_values(){
  Serial.print("A Input: ");
  Serial.print(input_A);
  Serial.print(", Output: ");
  Serial.print(output_A);
  Serial.print(", Target: ");
  Serial.println(target_A);
}

void print_speeds(){
  Serial.print("Left Speed: ");
  Serial.print(left_speed);
  Serial.print(", Right Speed: ");
  Serial.println(right_speed);
}
