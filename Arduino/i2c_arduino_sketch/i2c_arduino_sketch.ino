#include <Wire.h>

// L298N motor driver control pins
#define enA 9;
#define in1 8;
#define in2 7;
#define in3 5;
#define in4 4;
#define enB 3;

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

  // Initialize Wire for I2C
  Wire.begin(0x8);
  // Assign method to be executed on receiving data
  Wire.onReceive(receiveEvent);
}

// Method executed when receiving data. 
void receiveEvent(int count){
  Serial.println("--------------");

  // Set first byte to left wheel speed
  if (Wire.available()){
    int c = Wire.read();
    setLeftSpeed(c);;
  }

  // Set second byte to right wheel speed
  if (Wire.available()){
    int c = Wire.read();
    setRightSpeed(c);;
  }
}

void loop() {
  delay(100);
}

// Method to set left wheel speed.
void setLeftSpeed(int num){
  Serial.print("Left Speed: ");
  Serial.println(num);
  analogWrite(enB, num);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Method to set right wheel speed.
void setRightSpeed(int num){
  Serial.print("Right Speed: ");
  Serial.println(num);
  analogWrite(enA, num);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
