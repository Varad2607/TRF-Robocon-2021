/* Filename                 : Encoder.h
 * Name of Class            : Encoder
 * Name of Methods          : Encoder(), setRadius(), getCLKpin(), updateEncoder(), getCounter(), resetCounter()
 *                            enc_distance_PID()
 * Name of Member Variables : 
 */

#ifndef _Encoder_H_
#define _Encoder_H_

class Encoder {
    /**************************************************** IOPins **********************************************************/
    int _CLK; //CLK pin for the encoder object
    int _DT;  //DT pin for encoder object

    /**********************************************************************************************************************/

    /******************************************** Wheel Radius *************************************************************/
    float _radius = 4.9; //radius of the encoder

    /**********************************************************************************************************************/
    volatile int counter; //counter of the encoder object

  public:

   /********************************************* Distance Covered ************************************************************/  
    float currentDistance;  //current distance covered by the encoder

    /**********************************************************************************************************************/

    /*********************************************** PID Values **********************************************************/
    float error_distance; //P error
    float d_error_distance; //D error
    float i_error_distance; //I error
    float preverror_distance; //previous error

    //-------------------------------------------------------------------------------------------------------------------------------------------

   /*Function Name   :  constructor for class Encoder
    Input            :  CLKpin and DTpin
    Output           :  assign the clock pin and trigger pin to respective variable
    Logic            :  As the object is created the clock and trigger pin will be assigned and will be declared as input pins
    Example Call     :  Encoder(CLKpin, DTpin)
    */
    Encoder() {}
    Encoder(int CLKpin, int DTpin) {
      _CLK = CLKpin;
      _DT = DTpin;
      pinMode(DTpin, INPUT_PULLUP);
      pinMode(CLKpin, INPUT_PULLUP);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  setRadius
      Input            :  radius value from private variable
      Output           :  Directly assigns radius to another variable for easy use in program
      Logic            :  N.A.
      Example Call     :  setRadius(radius)
    */
    
    void setRadius(float radius){
      _radius = radius;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------
    /*
      Function Name    :  getCLKpin
      Input            :  ~none~
      Output           :  Directly assigns CLKpin to another variable for easy use in program
      Logic            :  N.A.
      Example Call     :  getCLKpin()
    */

    int getCLKpin() {
      return _CLK;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

     /*
      Function Name    :  updateEncoder
      Input            :  ~none~
      Output           :  counter: counts of encoder
      Logic            :  When the triger pin is set high with the clk pin then one is added to counter else 1 is minus from counter
      Example Call     :  updateEncoder()
    */

    void updateEncoder() {
      if (digitalRead(_DT) == 0) {
        counter++;
      }
      else {
        counter--;
      }
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  getCounter
      Input            :  ~none~
      Output           :  returns counter
      Logic            :  N.A.
      Example Call     :  getCounter()
    */

    int getCounter() {
      return counter;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  resetCounter()
      Input            :  ~none~
      Output           :  ~none~
      Logic            :  resets counter back to zero
      Example Call     :  resetCounter()
    */

    void resetCounter() {
      counter = 0;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  enc_distance_PID
      Input            :  Distance to travel and pwm vales( distance, kp, kd, ki)
      Output           :  base speed for the motor at every iteration (baseSpeed_dist)
      Logic            :  This methord is used to calculate pwm for Auto.h ie to reach a certain distance using autonomous control it is
                          calculated using PID constains and errors without any base                           
      Example Call     :  enc_distance_PID(distance, kp, kd, ki)
    */    

    int enc_distance_PID(float distance, float kp, float kd, float ki)
    {
      //PID calculations for Encoder
      int baseSpeed_dist;

      currentDistance = abs(0.010472 * counter * _radius);

      error_distance = distance - currentDistance;
      d_error_distance = preverror_distance - error_distance;

      if (error_distance < 10) {
        i_error_distance += error_distance;
      }

      baseSpeed_dist = kp * error_distance + kd * d_error_distance + ki * i_error_distance;

      preverror_distance = error_distance;
      return baseSpeed_dist;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------
};

#endif
