#include <AccelStepper.h>

// Defining the Motors 
AccelStepper motor1(AccelStepper::DRIVER, 3, 4, 5); //(step pin, direction pin, enable pin)
AccelStepper motor2(AccelStepper::DRIVER, 6, 7, 8);
AccelStepper motor3(AccelStepper::DRIVER, 9, 10, 11);

// Setting Motor Homing
const int home_delay = 2000; //ms
// Setting the Stepper Motors Speed and Acceleration
///////////// MOTOR ONE ///////////////
const int mtr1_maxspeed = 2000;
const int mtr1_acceleration = 25000;
//const int mtr1_speed = 500000;
//Return Conditions
const int mtr1_rmaxspeed = 1000;
const int mtr1_racceleration = 500;
////////////// MOTOR TWO //////////////
const int mtr2_maxspeed = 2000;
const int mtr2_acceleration = 25000;
//const int mtr2_speed = 500000;
//Return Conditions
const int mtr2_rmaxspeed = 1000;
const int mtr2_racceleration = 500;
//////////// MOTOR THREE ///////////////
const int mtr3_maxspeed = 2000;
const int mtr3_acceleration = 25000;
//const int mtr3_speed = 500000;
//Return Conditions
const int mtr3_rmaxspeed = 1000;
const int mtr3_racceleration = 500;

void setup() {
  //setting the serial communication 
  Serial.begin(9600);
  // Setting the maximum speed and acceleration of each motor
  motor1.setMaxSpeed(mtr1_maxspeed);
  motor1.setAcceleration(mtr1_acceleration);
  motor2.setMaxSpeed(mtr2_maxspeed);
  motor2.setAcceleration(mtr2_acceleration);
  motor3.setMaxSpeed(mtr3_maxspeed);
  motor3.setAcceleration(mtr3_acceleration);
}

void MoveMotors(int move1, int move2, int move3) {
  // Move the desired amount of steps 
  motor1.moveTo(-move1);
  motor2.moveTo(move2);
  motor3.moveTo(move3);

  // Run each motor until it reaches its target position
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
  delay(home_delay);
} 

void ReturnHome() {
  //Changing speed and acceleration slower
  motor1.setMaxSpeed(mtr1_rmaxspeed);
  motor1.setAcceleration(mtr1_racceleration);
  motor2.setMaxSpeed(mtr2_rmaxspeed);
  motor2.setAcceleration(mtr2_racceleration);
  motor3.setMaxSpeed(mtr3_rmaxspeed);
  motor3.setAcceleration(mtr3_racceleration);

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
  //Changing speed and acceleration slower
  motor1.setMaxSpeed(mtr1_maxspeed);
  motor1.setAcceleration(mtr1_acceleration);
  motor2.setMaxSpeed(mtr2_maxspeed);
  motor2.setAcceleration(mtr2_acceleration);
  motor3.setMaxSpeed(mtr3_maxspeed);
  motor3.setAcceleration(mtr3_acceleration);
  delay(1000);
}

//Connecting to Python and Calling Functions
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
