/* Filename                 : Manual.h
 * Name of Class            : Manual
 * Name of Methods          : forward(), backward(), left(), right(), CCW(), CW(), stop_bot()
 * Name of Member Variables : 
 *
 */

#ifndef _Manual_H_
#define _Manual_H_
#include "Encoder.h"
#include "MPU6050.h"
#include "Motor.h"
#include "move.h"

class Manual: public Move {
  public:
    /*
    Function Name    :  constructor for class Manual
    Input            :  pointers for class objects (MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3)
    Output           :  N.A.
    Logic            :  will pass on the pointers to the parent constrcutor which is explicitly called
    Example Call     :  Manual(MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3)
    */
    
    Manual(MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3): Move(mpu, encX, encY, motor1, motor2, motor3) {
      
    }
 
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  forward
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants (kp_enc_x_bwd, kd_enc_x_bwd, ki_enc_x_bwd)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have zero PWM (unless there is any lateral shift).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for backward function) and Motor 2 has zero PWM, the bot will move in forward direction. 
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  forward(fkp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_x_fwd, kd_enc_x_fwd, ki_enc_x_fwd)
    */
    
    void forward(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_x_fwd, float kd_enc_x_fwd, float ki_enc_x_fwd) //function for moving forward
    {
      mpuError();
      encXError();
      
      motor3_move->setDir();
      motor1_move->dirChange();

      pwm_1 = base + (kp1 * error + kd1 * (d_error) + kp_enc_x_fwd * error_enc_x);
      pwm_3 = base - (kp3 * error + kd3 * (d_error) + kp_enc_x_fwd * error_enc_x);
      pwm_2 = 0 * error_enc_x;


      if (error_enc_x < 0) { 
        motor2_move->setDir();
      }
      else {
        motor2_move->dirChange();
      }

      pwm_constrain(max_pwm, 50, max_pwm);
      disp();

      writeMotorPwm(pwm_1, pwm_2, pwm_3);

      preverror = error;
      S = 0;
      //Serial.println(error);
//      Serial.println("MPU");
//      Serial.println(error);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  backward
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants (kp_enc_x_fwd, kd_enc_x_fwd, ki_enc_x_fwd)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have zero PWM (unless there is any lateral shift).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for forward function) and Motor 2 has zero PWM, the bot will move in backward direction. 
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  backward(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_x_bwd, kd_enc_x_bwd, ki_enc_x_bwd)
    */

    void backward(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_x_bwd, float kd_enc_x_bwd, float ki_enc_x_bwd)
    {
      mpuError();
      encXError();

      motor3_move->dirChange(); //HIGH
      motor1_move->setDir();  //LOW

      pwm_1 = base - (kp1 * error + kd1 * (d_error) + kp_enc_x_bwd * error_enc_x);
      pwm_3 = base + (kp3 * error + kd3 * (d_error) + kp_enc_x_bwd * error_enc_x);
      pwm_2 = 0 * error_enc_x;

      if (error_enc_x < 0) {
        motor2_move->dirChange();
        Serial.print(" DIR2 ");
        Serial.print(" HIGH ");
      }
      else {
        motor2_move->setDir();
        Serial.print(" DIR2 ");
        Serial.print(" LOW ");
      }

      pwm_constrain(max_pwm, 50, max_pwm);

      disp();
      writeMotorPwm(pwm_1, pwm_2, pwm_3);

      preverror = error;
      S = 0;
    }
    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  left
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants (kp_enc_y_l, kd_enc_y_l, ki_enc_y_l)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have twice the PWM given to the other 2 motors (individually).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for right function) and Motor 2 has twice the PWM, the bot will move in left direction. 
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  left(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_y_l, kd_enc_y_l, ki_enc_y_l)
    */

    void left(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_y_l, float kd_enc_y_l, float ki_enc_y_l)
    {
      mpuError();
      encYError();

      motor3_move->setDir();
      motor1_move->setDir();
      motor2_move->setDir();


      pwm_1 = base / 2 - (kp1 * error + kd1 * (d_error)) + kp_enc_y_l * (error_enc_y) + 10;
      pwm_2 = base + (kp2 * error + kd2 * (d_error)) - kp_enc_y_l * abs(error_enc_y);
      pwm_3 = base / 2 + (kp3 * error + kd3 * (d_error)) - kp_enc_y_l * (error_enc_y);


      pwm_constrain(max_pwm, max_pwm, max_pwm);

      disp();
      writeMotorPwm(pwm_1, pwm_2, pwm_3);

      preverror = error;
      S = 0;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  right
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants (kp_enc_y_r, kd_enc_y_r, ki_enc_y_r)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have twice the PWM given to the other 2 motors (individually).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for left function) and Motor 2 has twice the PWM, the bot will move in right direction. 
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  right(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_y_r, kd_enc_y_r, ki_enc_y_r)
    */

    void right(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_y_r, float kd_enc_y_r, float ki_enc_y_r)
    {
      mpuError();
      encYError();
      
      motor3_move->dirChange();
      motor1_move->dirChange();
      motor2_move->dirChange();

      pwm_1 = base / 2 + (kp1 * error + kd1 * (d_error)) - kp_enc_y_r * (error_enc_y) + 10;
      pwm_2 = base - (kp2 * error + kd2 * (d_error)) - kp_enc_y_r * abs(error_enc_y);
      pwm_3 = base / 2 - (kp3 * error + kd3 * (d_error)) + kp_enc_y_r * (error_enc_y) + 10;

      disp();
      writeMotorPwm(pwm_1, pwm_2, pwm_3);

      preverror = error;
      S = 0;
    }

    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  CCW
       Input            :  ~none~
       Output           :  Motor 1, 2 and 3 will move with same pwm.
       Logic            :  While Motors 1, 2 and 3 have the same pwm (with directions exactly opposite to that for CW() function) the bot will move in CCW direction. 
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  CCW()
    */

    void CCW()
    {
      motor1_move->setDir();
      motor2_move->setDir();
      motor3_move->setDir();

      int base_CCW = 90;

      pwm_1 = base_CCW;
      pwm_2 = base_CCW;
      pwm_3 = base_CCW;

      writeMotorPwm(pwm_1, pwm_2, pwm_3);
    }
    
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  CW
       Input            :  ~none~
       Output           :  Motor 1, 2 and 3 will move with same pwm.
       Logic            :  While Motors 1, 2 and 3 have the same pwm (with directions exactly opposite to that for CCW() function) the bot will move in CW direction. 
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  CW()
    */

    void CW()
    {
      motor1_move->dirChange();
      motor2_move->dirChange();
      motor3_move->dirChange();

      int base_CW = 90;

      pwm_1 = base_CW;
      pwm_2 = base_CW;
      pwm_3 = base_CW + 15;

      writeMotorPwm(pwm_1, pwm_2, pwm_3);
    }
 
    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  stop_bot
       Input            :  ~none~
       Output           :  Motor 1, 2 and 3 will have 0 pwm
       Logic            :  PWM wiil be 0 so that bot will stop
       Example Call     :  stop_bot()
    */

    void stop_bot()
    {
      reset_yaw();
      writeMotorPwm(0, 0, 0);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

};

#endif
