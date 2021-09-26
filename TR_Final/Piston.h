/* Filename                 : Piston.h
   Name of Class            : Piston
   Name of Methods          : actuate_main(), lock_unlock(), stop_piston()
   Name of Member Variables :
*/

#ifndef _Piston_H_
#define _Piston_H_

class Piston
{
    int extract_piston;
    int retract_piston;
    int extract_lock;
    int retract_lock;

  public:

    //-------------------------------------------------------------------------------------------------------------------------------------------
    /*
      Function Name    :  constructor for class Piston
      Input            :  pin numbers for extracting and retracting pistons
      Output           :  N.A.
      Logic            :  will store the pin numbers for the extract-retract pins of various pistons in th mechanism
      Example Call     :  b1(52, 44, 50, 48)
    */

    Piston(int extract_piston, int retract_piston, int extract_lock, int retract_lock)
    {
      this->extract_piston = extract_piston;
      this->retract_piston = retract_piston;
      this->extract_lock = extract_lock;
      this->retract_lock = retract_lock;

      pinMode(extract_piston, OUTPUT);
      pinMode(retract_piston, OUTPUT);
      pinMode(extract_lock, OUTPUT);
      pinMode(retract_lock, OUTPUT);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------
    /*
      Function Name    :  shoot_arrow()
      Input            :  ~none~
      Output           :  ~none~
      Logic            :  the piston extracts and the restriction is disabled
                         (WHEN THE FLAG IS SET THE PISTON IS EXTRACTED)
      Example Call     :  shoot_arrow()
    */

    void shoot_arrow() {
      digitalWrite(extract_piston, LOW);
      digitalWrite(retract_piston, HIGH);
      delay(1000);

      digitalWrite(extract_lock, LOW);
      digitalWrite(retract_lock, HIGH);
      delay(200);
    }



    //-------------------------------------------------------------------------------------------------------------------------------------------
    /*
      Function Name    :  enable_restriction()
      Input            :  ~none~
      Output           :  ~none~
      Logic            :  the piston retracts and the restriction is enabled
                         (WHEN THE LOCK FLAG IS SET THE RESTRICTION IS ENABLED)
      Example Call     :  enable_restriction()
    */

    void enable_restriction() {
      digitalWrite(extract_piston, HIGH);
      digitalWrite(retract_piston, LOW);
      delay(1000);
      
      digitalWrite(extract_lock, HIGH);
      digitalWrite(retract_lock, LOW);
      delay(200);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------
    /*
      Function Name    :  stop_piston()
      Input            :  ~none~
      Output           :  ~none~
      Logic            :  sets all pins to low (default condition)
      Example Call     :  stop_piston()
    */

    void stop_piston()
    {
      digitalWrite(extract_piston, LOW);
      digitalWrite(retract_piston, LOW);
      digitalWrite(extract_lock, LOW);
      digitalWrite(retract_lock, LOW);
    }
};

#endif
