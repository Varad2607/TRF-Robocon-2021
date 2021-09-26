/* Filename                 : Auto.h
   Name of Class            : Auto
   Name of Methods          : Auto(), auto_forward(), auto_backward(), auto_left(), auto_right(), angle()
   Name of Member Variables :
*/

#ifndef _Auto_H_
#define _Auto_H_
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "move.h"
#include "Piston.h"

class Auto: public Move {
    //autonomous

    /**********************************************************************************************************************/
    Piston *b1_auto;
    Piston *b2_auto;

    /*********************************************** Output values of PID functions ***********************************************************/
    int baseSpeed_fwd;  //for forward direction
    int baseSpeed_bwd;  //for backward direction
    int baseSpeed_right;  //for the right direction
    int baseSpeed_left; //for the left direction

    /**********************************************************************************************************************/

    /*********************************************** PID values for angle rotation ***********************************************************/
    float error_angle;  //P error
    float preverror_angle;  //previous error
    float d_error_angle;  //D error
    float i_error_angle;  //I error

    /**********************************************************************************************************************/

    int left_flag = 1;
    int right_flag = 1;

  public:

    /*
      Function Name    :  constructor for class Auto
      Input            :  pointers for class objects (MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3)
      Output           :  N.A.
      Logic            :  will pass on the pointers to the parent constrcutor which is explicitly called
      Example Call     :  Auto(MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3)
    */

    Auto(MPU6050 *mpu, Encoder *encX, Encoder *encY, Motor *motor1, Motor *motor2, Motor *motor3, Piston *b1, Piston *b2): Move(mpu, encX, encY, motor1, motor2, motor3) {
      b1_auto = b1;
      b2_auto = b2;
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  auto_forward
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants for X encoder (kp_enc_x_fwd, kd_enc_x_fwd, ki_enc_x_fwd) and for Y encoder (kp_enc_auto_fwd, kd_enc_auto_fwd, ki_enc_auto_fwd)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have zero PWM (unless there is any lateral shift).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for backward function) and Motor 2 has zero PWM, the bot will move in forward direction.
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  auto_forward(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_x_fwd, kd_enc_x_fwd, ki_enc_x_fwd, kp_enc_auto_fwd, kd_enc_auto_fwd, ki_enc_auto_fwd, distance)
    */

    void auto_forward(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, double kp3, float kd3, float ki3, float kp_enc_x_fwd, float kd_enc_x_fwd, float ki_enc_x_fwd, float kp_enc_auto_fwd, float kd_enc_auto_fwd, float ki_enc_auto_fwd, float distance)
    {
      encY_move->currentDistance = 0;
      reset_yaw();
      while (encY_move->currentDistance < distance) {
        mpuError();
        encXError();

        motor3_move->setDir(); //HIGH
        motor1_move->dirChange();  //LOW

        baseSpeed_fwd = encY_move->enc_distance_PID(distance, kp_enc_auto_fwd, kd_enc_auto_fwd, ki_enc_auto_fwd);

        if (abs(encY_move->error_distance) > 10) {
          pwm_1 = baseSpeed_fwd + (kp1 * error + kd1 * (d_error) + kp_enc_x_fwd * error_enc_x);
          pwm_3 = baseSpeed_fwd - (kp3 * error + kd3 * (d_error) + kp_enc_x_fwd * error_enc_x);
          pwm_2 = 0 * error_enc_x;
        }
        else {
          stop_bot();
          break;
        }

        if (error_enc_x < 0) {
          motor2_move->setDir();
        }
        else {
          motor2_move->dirChange();
        }

        pwm_constrain(max_pwm, 50, max_pwm);
        Serial.print("error_distance_y ");
        Serial.print(encY_move->error_distance);
        disp();

        writeMotorPwm(pwm_1, pwm_2, pwm_3);

        preverror = error;

      }

    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
      Function Name    :  auto_backward
      Input            :  Proportionality (kp1, kp2, kp3), Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                          Encoder PID constants for X encoder (kp_enc_x_bwd, kd_enc_x_bwd, ki_enc_x_bwd) and for Y encoder (kp_enc_auto_bwd, kd_enc_auto_bwd, ki_enc_auto_bwd)
      Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have zero PWM (unless there is any lateral shift).
      Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for forward function) and Motor 2 has zero PWM, the bot will move in backward direction.
                          MPU and two XY Encoders are used for FEEDBACK of System.
                          PID is implemented for precise movement.
      Example Call     :  auto_backward(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_x_bwd, kd_enc_x_bwd, ki_enc_x_bwd, kp_enc_auto_bwd, kd_enc_auto_bwd, ki_enc_auto_bwd, distance)
    */

    void auto_backward(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_x_bwd, float kd_enc_x_bwd, float ki_enc_x_bwd, float kp_enc_auto_bwd, float kd_enc_auto_bwd, float ki_enc_auto_bwd, float distance)
    {
      encY_move->currentDistance = 0;
      reset_yaw();
      while (encY_move->currentDistance < distance) {
        mpuError();
        encXError();

        motor3_move->dirChange(); //HIGH
        motor1_move->setDir();  //LOW

        baseSpeed_bwd = encY_move->enc_distance_PID(distance, kp_enc_auto_bwd, kd_enc_auto_bwd, ki_enc_auto_bwd);

        if (abs(encY_move->error_distance) > 40) {
          pwm_1 = baseSpeed_bwd - (kp1 * error + kd1 * (d_error) + kp_enc_x_bwd * error_enc_x);
          pwm_3 = baseSpeed_bwd + (kp3 * error + kd3 * (d_error) + kp_enc_x_bwd * error_enc_x);
          pwm_2 = 0 * error_enc_x;
        }
        else {
          stop_bot();
          break;
        }

        if (error_enc_x < 0) {
          motor2_move->dirChange();
        }
        else {
          motor2_move->setDir();
        }

        pwm_constrain(max_pwm, 50, max_pwm);

        Serial.print("error_distance_y ");
        Serial.print(encY_move->error_distance);
        disp();

        writeMotorPwm(pwm_1, pwm_2, pwm_3);

        preverror = error;

      }

    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  auto_left
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants for Y Encoder (kp_enc_y_l, kd_enc_y_l, ki_enc_y_l), for X encoder (kp_enc_auto_left, kd_enc_auto_left, ki_enc_auto_left)
                           distance to be travelled (distance)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have twice the PWM given to the other 2 motors (individually).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for right function) and Motor 2 has twice the PWM, the bot will move in left direction.
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  auto_left(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_y_left, kd_enc_y_left, ki_enc_y_left, kp_enc_auto_left, kd_enc_auto_left, ki_enc_auto_left, distance)
    */

    void auto_left(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_y_left, float kd_enc_y_left, float ki_enc_y_left, float kp_enc_auto_left, float kd_enc_auto_left, float ki_enc_auto_left, float distance)
    {
      encX_move->currentDistance = 0;
      reset_yaw();
      while (encX_move->currentDistance < distance) {
        mpuError();
        encYError();

        motor3_move->setDir();
        motor1_move->setDir();
        motor2_move->setDir();

        baseSpeed_left = encX_move->enc_distance_PID(distance, kp_enc_auto_left, kd_enc_auto_left, ki_enc_auto_left);

        if (encX_move->error_distance > 20) {
          pwm_1 = (baseSpeed_left / 2) - (kp1 * error + kd1 * (d_error)) + kp_enc_y_left * (error_enc_y) + 10;
          pwm_2 = baseSpeed_left + (kp2 * error + kd2 * (d_error)) - kp_enc_y_left * abs(error_enc_y);
          pwm_3 = (baseSpeed_left / 2) + (kp3 * error + kd3 * (d_error)) - kp_enc_y_left * (error_enc_y) + 10;
        }
        else {
          Serial.println("In the else loop...!!");
          b1_auto->enable_restriction();
          b2_auto->enable_restriction();
          stop_bot();
        }

        pwm_constrain(max_pwm, max_pwm, max_pwm);

        //        Serial.print("error_distance_x ");
        //        Serial.print(encX_move->error_distance);
        //        disp();
        writeMotorPwm(pwm_1, pwm_2, pwm_3);

        preverror = error;

      }

    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  auto_right
       Input            :  Proportionality (kp1, kp2, kp3) Differential (kd1, kd2, kd3) and Integral (ki1, ki2, ki3) constants for each motor,
                           Encoder PID constants (kp_enc_y_right, kd_enc_y_right, ki_enc_y_right), for X encoder (kp_enc_auto_right, kd_enc_auto_right, ki_enc_auto_right)
       Output           :  Motor 1 and 3 will move with same pwm and Motor 2 will have twice the PWM given to the other 2 motors (individually).
       Logic            :  While Motors 1 and 3 have the same pwm (with directions exactly opposite to that for right function) and Motor 2 has twice the PWM, the bot will move in left direction.
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  auto_right(kp1, kd1, ki1, kp2, kd2, ki2, kp3, kd3, ki3, kp_enc_y_right, kd_enc_y_right, ki_enc_y_right, kp_enc_auto_right, kd_enc_auto_right, ki_enc_auto_right, distance)
    */


    void auto_right(float kp1, float kd1, float ki1, float kp2, float kd2, float ki2, float kp3, float kd3, float ki3, float kp_enc_y_right, float kd_enc_y_right, float ki_enc_y_right, float kp_enc_auto_right, float kd_enc_auto_right, float ki_enc_auto_right, float distance)

    {
      encX_move->currentDistance = 0;
      reset_yaw();
      while (encX_move->currentDistance < distance) {
        mpuError();
        encYError();

        motor3_move->dirChange();
        motor1_move->dirChange();
        motor2_move->dirChange();


        baseSpeed_right = encX_move->enc_distance_PID(distance, kp_enc_auto_right, kd_enc_auto_right, ki_enc_auto_right);

        if (encX_move->error_distance > 10) {
          pwm_1 = baseSpeed_right / 2 + (kp1 * error + kd1 * (d_error)) - kp_enc_y_right * (error_enc_y);
          pwm_2 = baseSpeed_right - (kp1 * error + kd1 * (d_error)) - kp_enc_y_right * abs(error_enc_y);
          pwm_3 = baseSpeed_right / 2 - (kp1 * error + kd1 * (d_error)) + kp_enc_y_right * (error_enc_y);
        }
        else {
          b1_auto->enable_restriction();
          b2_auto->enable_restriction();
          stop_bot();
          break;
        }

        pwm_constrain(max_pwm, max_pwm, max_pwm);

        Serial.print("error_distance_x ");
        Serial.print(encX_move->error_distance);
        disp();
        writeMotorPwm(pwm_1, pwm_2, pwm_3);

        preverror = error;
      }

    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

    /*
       Function Name    :  angle
       Input            :  MPU PID constants (kp_angle, kd_angle, ki_angle)
                           The target angle to reach (setpoint_angle)
                           The direction of rotation (CW/CCW)
       Output           :  Motor 1, 2 and 3 will move with same pwm.
       Logic            :  While Motors 1, 2 and 3 have the same pwm (with directions exactly opposite to that for CW() function) the bot will move in CCW direction.
                           MPU and two XY Encoders are used for FEEDBACK of System.
                           PID is implemented for precise movement.
       Example Call     :  angle(kp_angle, kd_angle, ki_angle, setpoint_angle, rotate_dir)
    */

    /*void angle(float kp_angle, float kd_angle, float ki_angle, float setpoint_angle, String rotate_dir) {
      reset_yaw();
      error_angle = setpoint_angle - abs(yaw);
      while (abs(error_angle) > 2) {
        mpuError();
        error_angle = setpoint_angle - abs(yaw);
        d_error_angle = abs(error_angle - preverror_angle);

        if (error_angle < 5) {
          i_error_angle += abs(error_angle);
        }

        if (rotate_dir == "CCW" && error_angle > 0 || (rotate_dir == "CW" && error_angle < 0)) {
          motor1_move->setDir();
          motor2_move->setDir();
          motor3_move->setDir();
        }
        else if (rotate_dir == "CW" && error_angle > 0 || (rotate_dir == "CCW" && error_angle < 0)) {
          motor1_move->dirChange();
          motor2_move->dirChange();
          motor3_move->dirChange();
        }

        pwm_1 = kp_angle * abs(error_angle) + kd_angle * (d_error_angle) + ki_angle * (i_error_angle);
        pwm_2 = kp_angle * abs(error_angle) + kd_angle * (d_error_angle) + ki_angle * (i_error_angle);
        pwm_3 = kp_angle * abs(error_angle) + kd_angle * (d_error_angle) + ki_angle * (i_error_angle);

        pwm_constrain(max_pwm, max_pwm, max_pwm);

        Serial.print("error angle ");
        Serial.print(error_angle);
        Serial.print(" ");

        disp();
        writeMotorPwm(pwm_1, pwm_2, pwm_3);

        preverror_angle = error_angle;
      }
      writeMotorPwm(0, 0, 0);
      }*/

    void reorient() {
      int setpoint_angle;
      String rotate_dir;
      float kp_angle = 0;
      float kd_angle = 0;
      float ki_angle = 0;
      setpoint_angle = 180;

      mpuError();

      if (178 < yaw && yaw < 182) {
        return;
      }
      else if (yaw > 180) {
        rotate_dir = "CW";
      }
      else if (yaw < 180) {
        rotate_dir = "CCW";
      }

      error_angle = abs(setpoint_angle - yaw);

      if(error_angle <= 20){
        kp_angle = 40/error_angle;
      }
      else if(error_angle > 20 && error_angle <= 40){
        kp_angle = 55/error_angle;
      }
      else if(error_angle > 40){
        kp_angle = 70/error_angle;
      }

      while (abs(error_angle) > 2 && abs(error_angle) > 0) {
        Serial.print("Reorient: ");
        Serial.println(kp_angle);
        mpuError();
        error_angle = setpoint_angle - abs(yaw);
        d_error_angle = abs(error_angle - preverror_angle);

        if (error_angle < 5) {
          i_error_angle += abs(error_angle);
        }

        if (rotate_dir == "CCW") {
          motor1_move->dirChange();
          motor2_move->dirChange();
          motor3_move->dirChange();
        }
        else if (rotate_dir == "CW") {
          motor1_move->setDir();
          motor2_move->setDir();
          motor3_move->setDir();
        }

        pwm_1 = kp_angle * abs(error_angle) + kd_angle * (d_error_angle) + ki_angle * (i_error_angle);
        pwm_2 = kp_angle * abs(error_angle) + kd_angle * (d_error_angle) + ki_angle * (i_error_angle);
        pwm_3 = kp_angle * abs(error_angle) + kd_angle * (d_error_angle) + ki_angle * (i_error_angle);

        pwm_constrain(70, 70, 70);

        Serial.print("Yaw: ");
        Serial.print(yaw);
        Serial.print(" Setpoint: ");
        Serial.print(setpoint_angle);
        Serial.print(" Error Angle: ");
        Serial.print(error_angle);
        Serial.print(" Rotate Direction: ");
        Serial.print(rotate_dir);
        disp();
        writeMotorPwm(pwm_1, pwm_2, pwm_3);

        preverror_angle = error_angle;
      }
      writeMotorPwm(0, 0, 0);
    }


    /*void flag_reset() {
      left_flag = 1;
      right_flag = 1;
      }

      void auto_left_seq(float dist1, float dist2, float dist3, float dist4) {
      if (left_flag == 1) {
        auto_left(1.1, 0, 0, 1, 0, 0, 1, 0, 0, 0.03, 0, 0, 2.4, 0, 0, dist1);
        left_flag = 2;
      }

      else if (left_flag == 2) {
        auto_left(1.1, 0, 0, 1.3, 0, 0, 1, 0, 0, 0.03, 0, 0, 1.35, 0, 0, dist2);
        left_flag = 3;
      }

      else if (left_flag == 3) {
        auto_left(1.1, 0, 0, 1.3, 0, 0, 1, 0, 0, 0.03, 0, 0, 1.35, 0, 0, dist3);
        left_flag = 4;
      }

      else if (left_flag == 4) {
        auto_left(1.1, 0, 0, 1, 0, 0, 1, 0, 0, 0.03, 0, 0, 2.4, 0, 0, dist4);
        left_flag = 1;
      }
      }

      void auto_right_seq(float dist1, float dist2, float dist3) {
      if (right_flag == 1) {
        auto_right(1.1, 0, 0, 1, 0, 0, 1, 0, 0, 0.03, 0, 0, 2.4, 0, 0, dist1);
        right_flag = 2;
      }

      else if (right_flag == 2) {
        auto_right(1.1, 0, 0, 1, 0, 0, 1, 0, 0, 0.03, 0, 0, 2.4, 0, 0, dist2);
        right_flag = 3;
      }

      else if (right_flag == 3) {
        auto_right(1.1, 0, 0, 1, 0, 0, 1, 0, 0, 0.03, 0, 0, 2.4, 0, 0, dist3);
        right_flag = 1;
      }
      }*/


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
      writeMotorPwm(0, 0, 0);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------
};

#endif
