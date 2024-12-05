#include <Wire.h>
#include <LiquidCrystal.h>

// Define potentiometer pin
#define POTENTIOMETER_PIN A0

// Define motor control pins (connected to L293D)
#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define MOTOR_ENA 11

// Define encoder pins
#define ENCODER_A 2
#define ENCODER_B 12
// Encoder variables
volatile int encoderPosition = 0;
int lastEncoded = 0;

// Define LCD pins
LiquidCrystal lcd(7, 8, 6, 5, 4, 3); // RS, EN, D4, D5, D6, D7

// Setup
void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.begin(16, 2); // Set up the LCD's number of columns and rows
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

  // Initialize potentiometer pin
  pinMode(POTENTIOMETER_PIN, INPUT);

  // Initialize motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);

  Serial.println("System Initialized.");
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

// Loop
void loop() {
  // Read potentiometer value
  int potValue = analogRead(POTENTIOMETER_PIN);

  // Map the potentiometer value to motor speed (-255 to 255)
  int motorSpeed = map(potValue, 0, 1023, -255, 255);

  // Determine motor direction and control motor
  String direction;
  if (motorSpeed > 140) {
    digitalWrite(MOTOR_IN1, HIGH);  // Motor rotates clockwise
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, map(motorSpeed, 142, 255, 0, 255)); // Forward speed
    direction = "FWD"; // Forward direction
  } 
  else if (motorSpeed < 110) {
   
     digitalWrite(MOTOR_IN1, LOW);  // Motor rotates counterclockwise
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, map(motorSpeed, 108, 0, 0, 255)); // Forward speed

    direction = "REV"; // Reverse direction
  } 
  else {
    // Stop the motor
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, 0);
    direction = "STOP";
  }

  // Display motor speed and direction on LCD
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(abs(motorSpeed)); // Display absolute speed
  lcd.print("   "); // Clear remaining characters if any

  lcd.setCursor(0, 1);
  lcd.print("Dir: ");
  lcd.print(direction);
  lcd.print("   "); // Clear remaining characters if any

  // Display encoder position
  lcd.setCursor(9, 1);
  lcd.print("Pos: ");
  lcd.print(encoderPosition);
  lcd.print("   "); // Clear remaining characters if any

  Serial.print("Motor Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Direction: ");
  Serial.print(direction);
  Serial.print(" | Encoder Position: ");
  Serial.println(encoderPosition);

  delay(300); // Delay for stability
}

// Encoder interrupt function
void updateEncoder() {
  int MSB = digitalRead(ENCODER_A);
  int LSB = digitalRead(ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  lastEncoded = encoded;
}
