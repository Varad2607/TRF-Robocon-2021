/* Filename                 : PFR.h
 * Name of Class            : PFR
 * Name of Methods          : pick_from_rack_2(), pick_from_rack_3(), pick_drop_arrow(), starting_position(), neutral_state(), align_arrow()
 * Name of Member Variables : 
 */

#ifndef _PFR_H_
#define _PFR_H_

class PFR
{
    int extract_piston;
    int retract_piston;
    int open_gripper;
    int close_gripper;
    int motor;
    int dir_motor;
    
    int gripper_flag = 0;
    int align_flag = 0;

  public:

    /*
    Function Name    :  constructor for class PFR
    Input            :  pin numbers for extracting and retracting pistons (int extract_p, int retract_p, int open_gripper, int close_gripper, int motor, int dir_motor)
    Output           :  N.A.
    Logic            :  will store the pin numbers for the extract-retract pins of various pistons in th mechanism 
    Example Call     :  p(A9, A10, 43, 42, 10, 31)
    */
    PFR(int extract_piston, int retract_piston, int open_gripper, int close_gripper, int motor, int dir_motor)
    {
      this->extract_piston = extract_piston;
      this->retract_piston = retract_piston;
      this->open_gripper = open_gripper;
      this->close_gripper = close_gripper;
      this->motor = motor;
      this->dir_motor = dir_motor;

      pinMode(extract_piston, OUTPUT);
      pinMode(retract_piston, OUTPUT);
      pinMode(open_gripper, OUTPUT);
      pinMode(close_gripper, OUTPUT);
      pinMode(motor, OUTPUT);
      pinMode(dir_motor, OUTPUT);
    }

 //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  pick_from_rack()
    Input            :  number of arrows
    Output           :  ~none~
    Logic            :  set the motor direction and the PWM value of the motor i.e. analogWrite the value stored in the variable to respective pin
    Example Call     :  pick_from_rack(2)
    */

    void pick_from_rack(int arrows, int kp, int kd, int ki)
    {
      int max_rotate_pwm;
      int base_rotate_pwm;
      int pwm;

      if(arrows = 0){
        base_rotate_pwm = 40;
        max_rotate_pwm = 60;     
      }
      if(arrows = 2){
        base_rotate_pwm = 40;
        max_rotate_pwm = 75;
      }
      if(arrows = 3){
        base_rotate_pwm = 40;
        max_rotate_pwm = 120;
      }

      //pwm = base_rotate_pwm + (kp1 * error + kd1 * (d_error) + ki * (i_error));
      digitalWrite(dir_motor, HIGH);
      analogWrite(motor, pwm);
    }

    /*void starting_position()
    {
      digitalWrite(dir_motor, LOW);
      analogWrite(motor, 55);
      delay(880);
      digitalWrite(dir_motor, HIGH);
      analogWrite(motor, 30);
      delay(100);
      analogWrite(motor, 0);
      delay(300);
    }
    
    void pick_from_rack_2()
    {
      digitalWrite(dir_motor, HIGH);
      analogWrite(motor, 75);
    }

    void pick_from_rack_3()
    {
      digitalWrite(dir_motor, HIGH);
      analogWrite(motor, 120);
    }*/

 //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  pick_drop_arrow()
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  depending upon whether the gripper flag is set it opens and closes the gripper
                        (WHEN THE GRIPPER FLAG IS SET THE GRIPPER IS CLOSED)
    Example Call     :  pick_drop_arrow()
    */
    
    void pick_drop_arrow()
    {
      if (gripper_flag == 0)
      {
        digitalWrite(open_gripper, LOW);
        delay(100);
        digitalWrite(close_gripper, HIGH);
        delay(300);
        gripper_flag = 1;
      }

      else if (gripper_flag == 1)
      {
        digitalWrite(open_gripper, HIGH);
        delay(100);
        digitalWrite(close_gripper, LOW);
        delay(300);
        gripper_flag = 0;
      }
    }

 //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  neutral_state()
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  sets all pins to low (the default condition)
    Example Call     :  neutral_state()
    */

    void neutral_state()
    {
      digitalWrite(open_gripper, LOW);
      digitalWrite(close_gripper, LOW);
      digitalWrite(extract_piston, LOW);
      digitalWrite(retract_piston, LOW);
      analogWrite(motor, 0);
    }

 //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  align_arrow()
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  depending upon whether the align flag is set it aligns the gripped arrows with the trays or brings them back to position
                        (WHEN THE ALIGN FLAG IS SET THE ARROWS ARE ALIGNED WITH THE TRAYS)
    Example Call     :  align_arrow()
    */
    

    void align_arrow()
    {
      if (align_flag == 0)
      {
        digitalWrite(extract_piston, HIGH);
        digitalWrite(retract_piston, LOW);
        delay(100);
        align_flag = 1;
      }
      else if (align_flag == 1)
      {
        digitalWrite(extract_piston, LOW);
        digitalWrite(retract_piston, HIGH);
        delay(100);
        align_flag = 0;
      }

    }
};

#endif
