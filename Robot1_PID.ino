#include "Arduino.h"
#include <QTRSensors.h>
//#include <SoftwareSerial.h>

extern "C" void __cxa_pure_virtual() {while (1);} //added this because of a compiling error

#define ledoff PORTB  &= ~(1 << 4)
#define ledon PORTB |= (1 << 4)

#define Kp .4 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 20 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define Ki 0.2

#define rightMaxSpeed 210 // max speed of the robot
#define leftMaxSpeed 210 // max speed of the robot
#define rightBaseSpeed 170 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 170  // this is the speed at which the motors should spin when the robot is perfectly on the line

#define NUM_SENSORS  2     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN   4     // emitter is controlled by digital pin 2

#define rightMotor 0
#define leftMotor 1

//SoftwareSerial mySerial(5, 0);
QTRSensorsAnalog qtra((unsigned char[]) {1, 3} ,NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
unsigned int sensorValues[NUM_SENSORS];

void wait();
void flashLED(uint8_t flashes);

void setup()
{
  //mySerial.begin(9600);

  //pinMode(rightMotor, OUTPUT);
  //pinMode(leftMotor, OUTPUT);
  ledoff;
  pinMode(EMITTER_PIN, OUTPUT);

  flashLED(3);
  delay(100);

  digitalWrite(EMITTER_PIN, HIGH);
for (int i = 0; i < 100; i++){ // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
   qtra.calibrate();
 }
  digitalWrite(EMITTER_PIN, LOW);
  delay(20);
  wait();
  //delay(2000); // wait for 2s to position the bot before entering the main loop
  flashLED(2);
    // comment out for mySerial printing

    /*for (int i = 0; i < NUM_SENSORS; i++)
    {
      mySerial.print(qtra.calibratedMinimumOn[i]);
      mySerial.print(' ');
    }
    mySerial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      mySerial.print(qtra.calibratedMaximumOn[i]);
      mySerial.print(' ');
    }
    mySerial.println();
    mySerial.println();
    */

  }

auto lastError = 0;
auto proportional = 0;
auto derivative = 0;
auto integral = 0;
auto derv = 0;
auto setPoint = 300; // 300 is the point between the two IR's
auto setPointright = 900;
void loop()
{
  unsigned int sensors[NUM_SENSORS];
  auto position = qtra.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  auto error = 0;
  if (position > 600){ //This is to make the line follower direction independent
    error = position - setPointright;
  }else{
    error = position - setPoint;
      }
  proportional = error * Kp;
  if (derv == 1){
  derivative = Kd * (error - lastError);
    }
    derv = 1;
  integral = Ki * (proportional + integral);

  auto motorSpeed = proportional + integral + derivative;
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  /*mySerial.print("  motorSpeed: ");
  mySerial.print(motorSpeed);
  mySerial.print("  Position: ");
  mySerial.print(position);
  mySerial.print("  Error: ");
  mySerial.print(error);
  mySerial.print("  rMotor: ");
  mySerial.print(rightMotorSpeed);
  mySerial.print("  lMotor: ");
  mySerial.println(leftMotorSpeed);
  delay(1000);*/

    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

     {
    analogWrite(rightMotor, rightMotorSpeed);
    analogWrite(leftMotor, leftMotorSpeed);
      }
}

void wait(){
	analogWrite(rightMotor, LOW);
	analogWrite(leftMotor, LOW);
}

void flashLED(uint8_t flashes){

	while(flashes){
		flashes--;
		ledon;
		delay(200);
		ledoff;
		if(flashes){ delay(500); }
	}

	while(flashes){
		flashes--;
		ledon;
		delay(100);
		ledoff;
		if(flashes){ delay(200); }
	}
}
