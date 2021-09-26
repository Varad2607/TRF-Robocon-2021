/* Filename                 : Move.h
 * Name of Class            : Move
 * Name of Methods          : mpuError(), encXError(),  encYError(), disp(), writeMotorPwm(), reset_yaw(), pwm_constrain()
 * Name of Member Variables : 
 */

#ifndef _Move_H_
#define _Move_H_
#include "Encoder.h"
#include "MPU6050.h"
#include "Motor.h"

int S = 0;
# define resPin 25

class Move{

  protected:
    /******************************************* Object Pointers ******************************************************/
    //pointers to objects that are passed in the constructors
    Encoder *encX_move;
    Encoder *encY_move;
    MPU6050 *mpu_move;
    Motor *motor1_move;
    Motor *motor2_move;
    Motor *motor3_move;

    /**********************************************************************************************************************/

    /*********************************************** PWM Values ***********************************************************/
    int pwm_1;  //will handle pid values for wheel 1
    int pwm_2;  //will handle pid values for wheel 2
    int pwm_3;  //will handle pid values for wheel 3
    int base = 220;  //base pwm
    int max_pwm = 240;  //maximum value up to which PWM can increase

    /**********************************************************************************************************************/

    /********************************************** MPU Variables *********************************************************/
    int yaw_old;  //the value read from the mpu
    int yaw;  //the value after subtracting the reference value
    int pre_yaw_0 = 3;
    int pre_yaw_360 = 3;
    int pre_yaw;
    int shift_yaw;  //reference value for reseting yaw

    //mpu PID
    float error;  //P error
    float preverror;  //previous error
    float d_error;  //D Error
    int setpoint = 0; //mpu set point (we want it to be stable on 0 degrees)
    int offSet = 0; //extra pwm if any wheel is lading behind
    int i_Error = 0;  //I error
    int _case;
    int flag_0 = 0;
    float _radius = 4.9;
    int cd;

    /**********************************************************************************************************************/
    
    /************************************* Encoder Variables ***********************************************/
    float error_enc_x; //for lateral shift
    float error_enc_y; //for lateral shift
    int setpoint_enc = 0; //setpoint for lateral shift (ideally lateral shift should be 0)

    //to store the values of x and y counters (for encoders) for class Move (and subclasses)
    volatile long int x_counter = 0;
    volatile long int y_counter = 0;

  public:

    Move() {
      //empty constructor for class move
    }

    /*
    Function Name    :  constructor for class Move
    Input            :  pointers for class objects (MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3)
    Output           :  will assign the pointer values to the pointer variables in the class
    Logic            :  the stored pointer values can be accessed throughout the class
    Example Call     :  Move(mpu, encX, encY, motor1, motor2, motor3)
    */
    
    Move(MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3) {
      mpu_move = mpu;
      encX_move = encX;
      encY_move = encY;
      motor1_move = motor1;
      motor2_move = motor2;
      motor3_move = motor3;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------

    /*
    Function Name    :  mpuError
    Input            :  ~none~
    Output           :  will calculate the values of shift_yaw and yaw as well as the proportional and differential errors 
    Logic            :  will assign the pointer values to the pointer variables in the class so that they can be accessed throughout the class
    Example Call     :  Move(mpu, encX, encY, motor1, motor2, motor3)
    */

    void mpuError() {
      //reads the MPU value and shifts it according to our reference value, also calculates the D error
      yaw_old = mpu_move->mpuRead();
      yaw = yaw_old - shift_yaw;

      error = setpoint - yaw;
      d_error = error - preverror;
      
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------
    
    /*
      Function Name    :  encXError
      Input            :  ~none~
      Output           :  reads the X counter value and calculates the error w.r.t. setpoint and calculates error value for X encoder
      Logic            :  N.A.
      Example Call     :  encXError()
    */

    void encXError() {
      x_counter = encX_move->getCounter();
      error_enc_x = setpoint_enc - x_counter;
      cd = abs(0.010472 * x_counter * _radius);
    }

    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  encYError
      Input            :  ~none~
      Output           :  reads the Y counter value and calculates the error w.r.t. setpoint and calculates error value for Y encoder
      Logic            :  N.A.
      Example Call     :  encYError()
    */
    
    void encYError() {
      //reads the Y counter value and calculates the error w.r.t. setpoint
      y_counter = encY_move->getCounter();
      error_enc_y = setpoint_enc - y_counter;
      cd = abs(0.010472 * y_counter * _radius);
    }

    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  disp
      Input            :  ~none~
      Output           :  displays all the variables
      Logic            :  N.A.
      Example Call     :  disp()
    */
    
    void disp() {
      Serial.print(" yaw ");
      Serial.print(yaw);
      Serial.print(" yaw_old ");
      Serial.print(yaw_old);
      Serial.print(" error ");
      Serial.print(error);
      Serial.print(" x encoder ");
      Serial.print(x_counter);
      Serial.print(" y encoder ");
      Serial.print(y_counter);
      Serial.print(" error encoder ");
      Serial.print(error_enc_x);
      Serial.print(" pwm1 ");
      Serial.print(pwm_1);
      Serial.print(" pwm2 ");
      Serial.print(pwm_2);
      Serial.print(" pwm3 ");
      Serial.println(pwm_3);
    }

    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  writeMotorPwm
      Input            :  pwm value for each motor (int p1, int p2, int p3)
      Output           :  passes the pwm values to the motor object function for analogWrite()
      Logic            :  N.A.
      Example Call     :  writeMotorPwm(p1, p2, p3)
    */

    void writeMotorPwm(int p1, int p2, int p3) {
      motor1_move->setPwm(p1);
      motor2_move->setPwm(p2);
      motor3_move->setPwm(p3);
    }

    
    //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
      Function Name    :  reset_yaw
      Input            :  ~none~
      Output           :  yaw_old is yaw recieved from mpu, stores this value as current reference value, resets encoder counter values
      Logic            :  we need to reset the mpu value every time the stop function is called so that we can now operate the bot w.r.t the current position as the point of reference
      Example Call     :  reset_yaw()
    */

    void reset_yaw() {
      yaw_old = mpu_move->mpuRead();
      //yaw = yaw_old;
      shift_yaw = yaw_old;
      yaw = yaw_old - shift_yaw;     
      
      encX_move->resetCounter();
      encY_move->resetCounter();
           
      //Serial.println("Not restart");
    }

    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  pwm_constrain
      Input            :  maximum pwm value for each motor (int max1, int max2, int max3)
      Output           :  maps and constrains the value of pwm1, pwm2, and pwm3 within the given range (0, max)
      Logic            :  PWM has can range between 0 to 255 and therefore needs to be mapped and/or constrained so as to not exceed that range
      Example Call     :  pwm_constrain(int max1, int max2, int max3)
    */

    void pwm_constrain(int max1, int max2, int max3) {
      //pwm is constrained in the range of 0 to the passed max value
      //parameters: maximum values of pwm for each value
      pwm_1 = constrain(abs(pwm_1), 0, max1);
      pwm_2 = constrain(abs(pwm_2), 0, max2);
      pwm_3 = constrain(abs(pwm_3), 0, max3);
    }

    //------------------------------------------------------------------------------------------------------------------------------------

    void mpu_reset(){
      digitalWrite(resPin, LOW);
      delay(100);    
      //Serial.println("Reset");
    }

};

#endif
