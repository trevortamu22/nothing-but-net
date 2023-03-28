#include <AccelStepper.h>

// Defining the Motors 
AccelStepper motor1(AccelStepper::DRIVER, 3, 4); //(step pin, direction pin)
AccelStepper motor2(AccelStepper::DRIVER, 6, 7);
AccelStepper motor3(AccelStepper::DRIVER, 9, 10);

void setup() {
  //setting the serial communication 
  Serial.begin(9600);
  // Setting the maximum speed and acceleration of each motor
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(2000);
  motor2.setAcceleration(1000);
  motor3.setMaxSpeed(2000);
  motor3.setAcceleration(1000);
}

void MoveMotors(int move1, int move2, int move3) {
  // Move the desired amount of steps 
  motor1.moveTo(move1);
  motor2.moveTo(move2);
  motor3.moveTo(move3);

  // Run each motor until it reaches its target position
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
  delay(2000);
} 

void ReturnHome() {
  //Changing speed and acceleration slower
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);
  motor3.setMaxSpeed(1000);
  motor3.setAcceleration(500);

  // Move each motor in the opposite direction to return to home 
  motor1.moveTo(0);
  motor2.moveTo(0);
  motor3.moveTo(0);

  // Run each motor until it reaches its target position
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
  while (1) {
  }
}

//Connecting to Python and calling functions
void loop() {
  if (Serial.available() > 0) {
    //Reading the incoming command from python 
    String command = Serial.readStringUntil('\n');
    if (command == "Move") {
      int move1 = Serial.parseInt();
      int move2 = Serial.parseInt();
      int move3 = Serial.parseInt();
      MoveMotors(move1, move2, move3);
    } else if (command == "ReturnHome") {
      ReturnHome();
    }
  }
}
