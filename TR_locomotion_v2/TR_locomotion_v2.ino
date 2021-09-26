#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Move.h"
#include "Manual.h"
#include "Piston.h"
#include "PFR.h"
#include "Auto.h"


/***************************************** PFR ******************************************/
PFR p(A9, A10, 43, 42, 10, 31);

/***************************************** Crossbow ******************************************/
char var = 's';
Piston b1(52, 44, 50, 48);
Piston b2(47, 45, 49, 46);

/*--------TIP Board arrangement---------

 * TIP Board 1 (central):- 
 * 44   42   A9   A10  A11
 * 46   48   50   52   GND
 * ---- PFR + Left Crossbow + Right Crossbow (retract_lock) -----
 * 
 * 
 * TIP Board 2 (edge):-
 * 45   47   49   51   53
 * 43   41   39   37   GND
 * ---- PFR (open_gripper) + Right Crossbow  -----

-----------------------------------------*/


/***************************************** MPU led correction and Bluetooth variables ***********************************/
char d; //character to store the value sent by bluetooth

/**************************************** Encoder Objects ***************************************************************/
Encoder X(3, 29); //Encoder X object
Encoder Y(2, 27); //Encoder Y object

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
Auto AUTO(&MPU, &X, &Y, &M1, &M2, &M3); //object of autonomous class

void loop()
{

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
      MANU.left(1.1, 0, 0, 1, 0, 0, 1, 0, 0, 0.03, 0, 0);
      break;
    case 'F':
      MANU.forward(1, 0, 0, 0, 0, 0, 0.8, 0, 0, 0, 0, 0);
      break;
    case 'B':
      MANU.backward(1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
      break;
    case 'C':
      MANU.mpu_reset();
      break;
    case 'M':
      MANU.CCW();
      break;
    case 'N':
      MANU.CW();
      break;
    case 'U':
      p.pick_from_rack(0, 0, 0, 0);
      break;
    case 'P':
      p.pick_from_rack(2, 0, 0, 0);
      break;
    case 'O':
      //AUTO.auto_forward(1.5, 0.2, 0, 0, 0, 0, 2, 0, 0, 0.3, 0.2, 0, 0.6, 1, 0, 200);
      p.pick_from_rack(3, 0, 0, 0);
      break;
    case 'I':
      p.pick_drop_arrow();
      break;
    case 'T':
      p.align_arrow();
      break;
    case 'Q':
      b1.lock_unlock();
      break;
    case 'W':
      b1.actuate_main();
      break;
    case 'E':
      b2.lock_unlock();
      break; 
    case 'J':
      b2.actuate_main();
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
