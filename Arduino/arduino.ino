#include <Servo.h>

Servo panServo;

void setup() {

  Serial.begin(9600);

  panServo.attach(9);

  panServo.write(90);
}


void loop() {

  if (Serial.available()) {

    int servoAngle = Serial.parseInt();

    servoAngle = constrain(
      servoAngle,
      20,
      160
    );

    panServo.write(
      servoAngle
    );
  }
}