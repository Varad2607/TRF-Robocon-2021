#ifndef _piston_H_
#define _piston_H_

class piston
{
    int extract_piston;
    int retract_piston;
    int extract_lock;
    int retract_lock;

  public:

    int flag1 = 0;
    int flag2 = 0;

  public:
    piston(int extract_piston, int retract_piston, int extract_lock, int retract_lock)
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

    void extract_main()
    {
      digitalWrite(extract_piston, HIGH);
      digitalWrite(retract_piston, LOW);
      delay(2000);
      Serial.println("pulled");
      flag2 = 1;
    }

    void retract_main()
    {
      digitalWrite(extract_piston, LOW);
      digitalWrite(retract_piston, HIGH);
      delay(2000);
      flag2 = 0;
    }


    void lock()
    {
      digitalWrite(extract_lock,HIGH);
      digitalWrite(retract_lock,LOW);
      delay(200);
      flag1 = 1;
      }
     
     void un_lock()
     {
      digitalWrite(extract_lock,LOW);
      digitalWrite(retract_lock,HIGH);
      delay(200);
      flag1 = 0; 
      
      }
    void stop_piston()
    {
      digitalWrite(extract_piston, LOW);
      digitalWrite(retract_piston, LOW);
      digitalWrite(extract_lock, LOW);
      digitalWrite(retract_lock, LOW);
    }
};

#endif
