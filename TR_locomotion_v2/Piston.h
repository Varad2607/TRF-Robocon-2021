/* Filename                 : Piston.h
 * Name of Class            : Piston
 * Name of Methods          : actuate_main(), lock_unlock(), stop_piston()
 * Name of Member Variables : 
 */
 
#ifndef _Piston_H_
#define _Piston_H_

class Piston
{
    int extract_piston;
    int retract_piston;
    int extract_lock;
    int retract_lock;

    int piston_flag = 0;
    int lock_flag = 0;

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
    Function Name    :  actuate_main()
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  depending upon whether the main piston flag is set it extracts or retracts the piston
                        (WHEN THE FLAG IS SET THE PISTON IS EXTRACTED)
    Example Call     :  actuate_main()
    */

    void actuate_main(){
      
     if(piston_flag == 0){
        digitalWrite(extract_piston, HIGH);
        digitalWrite(retract_piston, LOW);
        delay(2000);
        Serial.println("pulled");
        piston_flag = 1;
      }
      
      if(piston_flag == 1){
        digitalWrite(extract_piston, LOW);
        digitalWrite(retract_piston, HIGH);
        delay(2000);
        piston_flag = 0;
      }
      
    }

 //-------------------------------------------------------------------------------------------------------------------------------------------
   /*
    Function Name    :  lock_unlock()
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  depending upon whether the lock flag is set it opens or closes restriction
                        (WHEN THE LOCK FLAG IS SET THE RESTRICTION IS ENABLED)
    Example Call     :  lock_unlock()
    */

    void lock_unlock(){
      
     if(lock_flag == 0){
        digitalWrite(extract_lock, HIGH);
        digitalWrite(retract_lock, LOW);
        delay(200);
        lock_flag = 1;
      }
      
      if(lock_flag == 1){
        digitalWrite(extract_lock, LOW);
        digitalWrite(retract_lock, HIGH);
        delay(200);
        lock_flag = 0;
      }
      
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
