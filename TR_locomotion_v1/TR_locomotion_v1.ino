#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Move.h"
#include "Manual.h"
#include "piston.h"
#include "pfr.h"


/***************************************** PFR ******************************************/
pfr p(A9, A10, 43, 42, 6, 28);

/***************************************** Crossbow ******************************************/
char var = 's';
piston b1(52, 44, 50, 48);
piston b2(47, 45, 49, 46);

/***************************************** MPU led correction and Bluetooth variables ***********************************/
char d; //character to store the value sent by bluetooth

/**************************************** Encoder Objects ***************************************************************/
Encoder X(3, 29); //Encoder X object
Encoder Y(2, 27); //Encoder Y object
/************************************************************************************************************************/

/********************************************* Motor Objects ************************************************************/
Motor M1(9, 34, LOW);
Motor M2(7, 30, LOW);
Motor M3(8, 32, LOW);

/******************************************** MPU Object ****************************************************************/

MPU6050 MPU;
/************************************************************************************************************************/

void updateEncoderX() {
  X.updateEncoder();
}
void updateEncoderY() {
  Y.updateEncoder();
}

void setup() {
  digitalWrite(resPin, HIGH);
  pinMode(resPin, OUTPUT);
//  pinMode(A8, OUTPUT);
//  digitalWrite(A8, HIGH);

  int CLK1 = X.getCLKpin(); //the value of the clock pin for Encoder X
  int CLK2 = Y.getCLKpin(); //the value of the clock pin for Encoder Y

  attachInterrupt(digitalPinToInterrupt(CLK1), updateEncoderX, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK2), updateEncoderY, RISING);

  Serial2.begin(115200);  //Serial port for reading mpu values
  Serial.begin(115200);
  Serial3.begin(115200);  //Serial port for reading values transmitted by bluetoooth
  Serial1.begin(115200);
}


Manual MANU(&MPU, &X, &Y, &M1, &M2, &M3); //object of manual class


void loop()
{
  //  led = MPU.mpuRead();
  //  led = map(led, -180, 179, 0, 50);
  //  Serial.println(led);
  //  analogWrite(led_pin, led);

  digitalWrite(resPin, HIGH); //pin for MPU Board reset

  if (Serial3.available() > 0)
  {
    d = Serial3.read(); // TX1 = 18, RX1 = 19
  }

  Serial.println(d);
  switch (d) {
    case 'R':
      MANU.right(1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0);
      break;
    case 'L':
      MANU.left(1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0);
      break;
    case 'F':
      MANU.forward(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      break;
    case 'B':
      MANU.backward(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      break;
    case 'T':
      MANU.mpu_reset();
      break;
    case 'x':
      /*************************/
      break;
    case 'M':
      MANU.CCW();
      break;
    case 'N':
      MANU.CW();
      break;
    case 'O':
      p.pick_from_rack_3();
      break;
    case 'U':
      p.starting_position();
      break;
    case 'I':
      p.pick_drop_arrow();
      break;
    case 'Q':
      if (b1.flag1 == 0) {
        b1.lock();
      }

      else {
        b1.un_lock();
      }
      break;
    case 'W':
      if (b1.flag2 == 0) {
        b1.extract_main();
      }

      else {
        b1.retract_main();
      }
      break;

    case 'E':
      if (b2.flag1 == 0) {
        b2.lock();
      }

      else {
        b2.un_lock();
      }
      break;
    case 'J':
      if (b2.flag2 == 0) {
        b2.extract_main();
      }

      else {
        b2.retract_main();
      }
      break;

    case 'P':
      p.pick_from_rack_2();
      break;

    default:
      MANU.stop_bot();
      X.resetCounter();
      Y.resetCounter();
      b1.stop_piston();
      b2.stop_piston();
      p.neutral_state();
      break;
  }

}
