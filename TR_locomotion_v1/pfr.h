#ifndef _pfr_H_
#define _pfr_H_

class pfr
{
    int extract_p;
    int retract_p;
    int open_gripper;
    int close_gripper;
    int motor;
    int dir_motor;
    int flag = 0;
    int flag1 = 0;
    int flag2 = 0;

  public:
    pfr(int extract_p, int retract_p, int open_gripper, int close_gripper, int motor, int dir_motor)
    {
      this->extract_p = extract_p;
      this->retract_p = retract_p;
      this->open_gripper = open_gripper;
      this->close_gripper = close_gripper;
      this->motor = motor;
      this->dir_motor = dir_motor;

      pinMode(extract_p, OUTPUT);
      pinMode(retract_p, OUTPUT);
      pinMode(open_gripper, OUTPUT);
      pinMode(close_gripper, OUTPUT);
      pinMode(motor, OUTPUT);
      pinMode(dir_motor, OUTPUT);
    }

    void pick_from_rack_2()
    {
      align_arrow();
      delay(500);
      rotate_motor_2();
      delay(2000);
      drop_arrow();
      delay(2000);
      rotate_motor_2();
      delay(1000);
      align_arrow();
    }

    void pick_from_rack_3()
    {
      align_arrow();
      delay(500);
      rotate_motor_3();
      delay(2000);
      drop_arrow();
      delay(2000);
      rotate_motor_3();
      delay(500);
      align_arrow();
    }

    void pick_drop_arrow()
    {
      //close the gripper
      if (flag1 == 0)
      {
        digitalWrite(open_gripper, LOW);
        delay(100);
        digitalWrite(close_gripper, HIGH);
        delay(300);
        flag1 = 1;
      }

      //open the gripper
      else if (flag1 == 1)
      {
        digitalWrite(open_gripper, HIGH);
        delay(100);

        digitalWrite(close_gripper, LOW);
        delay(300);
        flag1 = 0;
      }
    }

    void starting_position()
    {
      digitalWrite(dir_motor, LOW);
      analogWrite(motor, 60);
      delay(880);
      digitalWrite(dir_motor, HIGH);
      analogWrite(motor, 30);
      delay(100);
      analogWrite(motor, 0);
    }

    void neutral_state()
    {
      digitalWrite(open_gripper, LOW);
      digitalWrite(close_gripper, LOW);
      digitalWrite(extract_p, LOW);
      digitalWrite(retract_p, LOW);
      analogWrite(motor, 0);
    }

  private:
    void align_arrow()
    {
      if (flag2 == 0)
      {
        digitalWrite(extract_p, HIGH);
        digitalWrite(retract_p, LOW);
        delay(100);
      }
      else if (flag2 == 1)
      {
        digitalWrite(extract_p, LOW);
        digitalWrite(retract_p, HIGH);
        delay(100);
      }

    }

    void rotate_motor_2()
    {
      //rotate motor in one direction
      if (flag == 0)
      {
        digitalWrite(dir_motor, HIGH);
        analogWrite(motor, 80);
        delay(900);
        analogWrite(motor, 40);
        delay(600);
        digitalWrite(dir_motor, LOW);
        analogWrite(motor, 30);
        delay(200);
        analogWrite(motor, 0);
      }

      //rotate motor in another direction
      else if (flag == 1)
      {
        digitalWrite(dir_motor, LOW);
        analogWrite(motor, 80);
        delay(900);
        analogWrite(motor, 40);
        delay(600);
        digitalWrite(dir_motor, HIGH);
        analogWrite(motor, 20 );
        delay(100);
        analogWrite(motor, 0);
        flag = 0;
        flag2 = 1;
      }
    }

    void rotate_motor_3()
    {
      //rotate motor in one direction
      if (flag == 0)
      {
        digitalWrite(dir_motor, HIGH);
        analogWrite(motor, 135);
        delay(850);
        digitalWrite(dir_motor, LOW);
        analogWrite(motor, 0);
        delay(200);
        analogWrite(motor, 0);
      }

      //rotate motor in another direction
      else if (flag == 1)
      {
        digitalWrite(dir_motor, LOW);
        analogWrite(motor, 90);
        delay(1000);
        digitalWrite(dir_motor, HIGH);
        analogWrite(motor, 60);
        delay(100);
        analogWrite(motor, 0);
        flag = 0;
        flag2 = 1;
      }
    }

    void drop_arrow()
    {
      digitalWrite(open_gripper, HIGH);
      digitalWrite(close_gripper, LOW);
      //delay(100);
      flag = 1;
    }
};

#endif
