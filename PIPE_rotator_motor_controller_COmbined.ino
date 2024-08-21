#include<AFMotor.h>
#define LEDPIN 13
#define SENSORPIN 17

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

int ByteReceived;
int motorspeed = 0;
int sensorState = 0, lastState = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("enter f=FORWARD ,b= BACKWARD ,s=BRAKE ,+/- = SPEED");
  Serial.println("enter 1-6 to turn on and off the devices");
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(SENSORPIN, INPUT);
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup

  motor1.setSpeed(motorspeed);
  motor2.setSpeed(motorspeed);
}

void loop() {// read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);

  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  if (sensorState == LOW) {
    // turn LED on:
    digitalWrite(LEDPIN, HIGH);
  }
  else {
    // turn LED off:
    digitalWrite(LEDPIN, LOW);
  }

  if (sensorState && !lastState) {
    Serial.println("Unbroken");
    motor1.run(BRAKE);
    delay(10);
    motor1.run(RELEASE);
    delay(10);
    motor1.run(BRAKE);
    delay(10);
    motor2.run(BRAKE);
    delay(10);
    motor2.run(RELEASE);
    delay(10);
    motor2.run(BRAKE);
    delay(10);
  }
  if (!sensorState && lastState) {
    Serial.println("Broken");
    motor1.run(FORWARD);
    delay(10);
    motor1.run(RELEASE);
    delay(10);
    motor1.run(FORWARD);
    delay(10);
    motor2.run(BACKWARD);
    delay(10);
    motor2.run(RELEASE);
    delay(10);
    motor2.run(BACKWARD);
    delay(10);
  }
  lastState = sensorState;
  
  if (Serial.available() > 0)
  {
    ByteReceived = Serial.read();
    Serial.print(char(ByteReceived));
    if (ByteReceived == '+')
    {
      motor1.setSpeed(motorspeed = motorspeed + 10);
      motor2.setSpeed(motorspeed = motorspeed + 10);

    }
    if (ByteReceived == '-')
    {
      motor1.setSpeed(motorspeed = motorspeed - 10);
      motor2.setSpeed(motorspeed = motorspeed - 10);

    }
    if (ByteReceived == 'f') // Single Quote! This is a character.
    {
      motor1.run(FORWARD);
      delay(10);
      motor1.run(RELEASE);
      delay(10);
      motor1.run(FORWARD);
      delay(10);
      motor2.run(BACKWARD);
      delay(10);
      motor2.run(RELEASE);
      delay(10);
      motor2.run(BACKWARD);
      delay(10);
    }
    if (ByteReceived == 'b') // Single Quote! This is a character.
    {
      motor1.run(BACKWARD);
      delay(10);
      motor1.run(RELEASE);
      delay(10);
      motor1.run(BACKWARD);
      delay(10);
      motor2.run(FORWARD);
      delay(10);
      motor2.run(RELEASE);
      delay(10);
      motor2.run(FORWARD);
      delay(10);
    }
    if (ByteReceived == 's') // Single Quote! This is a character.
    {
      motor1.run(BRAKE);
      delay(10);
      motor1.run(RELEASE);
      delay(10);
      motor1.run(BRAKE);
      delay(10);
      motor2.run(BRAKE);
      delay(10);
      motor2.run(RELEASE);
      delay(10);
      motor2.run(BRAKE);
      delay(10);
    }
    if (ByteReceived == '1')
    {
      digitalWrite(14, HIGH);

    }
    if (ByteReceived == '2')
    {
      digitalWrite(14, LOW);

    }
    if (ByteReceived == '3')
    {
      digitalWrite(15, HIGH);

    }
    if (ByteReceived == '4')
    {
      digitalWrite(15, LOW);

    }
    if (ByteReceived == '5')
    {
      digitalWrite(16, HIGH);

    }
    if (ByteReceived == '6')
    {
      digitalWrite(16, LOW);

    }
  }
}
