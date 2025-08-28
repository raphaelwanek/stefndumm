#include <Arduino.h>
#include <Servo.h>

// Motor A control pins
#define ENA 5       // PWM speed control for motor A
#define IN1 8       // Direction control pin 1 for motor A
#define IN2 9       // Direction control pin 2 for motor A

// Motor B control pins
#define ENB 6       // PWM speed control for motor B
#define IN3 10      // Direction control pin 1 for motor B
#define IN4 11      // Direction control pin 2 for motor B

// LED pin (optional visual indicator)
#define LED 12

int MAX_SPEED = 255;                // Maximum motor speed (PWM value)

Servo servus;                       // Servo motor for steering
int lastSteeringAngle = 90;         // Last angle sent to the servo
int tolerance = 2;                  // Minimum difference required to update steering
float pitch;                        // Variable to store pitch value from remote

#define BT Serial                   // Alias for Serial communication over Bluetooth (HC-05)

void setup() {
  servus.attach(3);                 // Attach servo to digital pin 3
  servus.write(90);                 // Initialize steering to center position

  pinMode(LED, OUTPUT);             // Set LED pin as output
  digitalWrite(LED, LOW);          // Turn LED off initially

  // Set motor pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  BT.begin(115200);                 // Begin Bluetooth communication at 115200 baud

  BT.println("Car Ready");          // Print startup message via Bluetooth
}

// Sets motor speed and direction based on signed speed value
void setMotors(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);  // Limit speed to valid range

  if (speed > 0) {
    // Move forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    // Move backward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Set PWM speed
  analogWrite(ENA, abs(speed));
  analogWrite(ENB, abs(speed));
}

void loop() {
  // Check if Bluetooth data is available
  while (BT.available()) {
    // Read one full message line (ending with \n)
    String line = BT.readStringUntil('\n');

    // If button press detected (e.g., "BUTTON:1")
    if (line.indexOf("BUTTON:1") >= 0) {
      digitalWrite(LED, HIGH);     // Turn LED on
    } else {
      digitalWrite(LED, LOW);      // Turn LED off
    }

    // Extract pitch value from message
    int pitchIndex = line.indexOf("PITCH:");
    if (pitchIndex >= 0) {
      int commaIndex1 = line.indexOf(',', pitchIndex);
      if (commaIndex1 == -1) commaIndex1 = line.length();
      String pitchString = line.substring(pitchIndex + 6, commaIndex1);
      pitch = pitchString.toFloat(); // Convert pitch string to float

      // Map pitch angle to steering angle (servo range)
      int steeringAngle = map(pitch, -75, 75, 60, 120);
      steeringAngle = constrain(steeringAngle, 0, 180);

      // Update servo only if the angle changed significantly
      if (abs(steeringAngle - lastSteeringAngle) > tolerance) {
        servus.write(steeringAngle);
        lastSteeringAngle = steeringAngle;
      }
    }

    // Extract roll value from message
    int rollIndex = line.indexOf("ROLL:");
    if (rollIndex >= 0) {
      int commaIndex2 = line.indexOf(',', rollIndex);
      String rollString = line.substring(rollIndex + 5, commaIndex2);
      float roll = rollString.toFloat(); // Convert roll string to float

      // Map roll angle to motor speed
      int speed = map(roll, -75, 75, -MAX_SPEED, MAX_SPEED);
      speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

      setMotors(speed); // Set motors with calculated speed
    }

    delay(10); // Small delay to avoid overloading the loop
  }
}