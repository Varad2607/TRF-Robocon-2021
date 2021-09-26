/* Filename                 : Motor.h
 * Name of Class            : Motor
 * Name of Methods          : setPwm(), setDir(), dirChange()
 * Name of Member Variables : 
 *
 */
 
#ifndef _Motor_H_
#define _Motor_H_

class Motor {

  /*********************************************** PWM And Dir Pins ****************************************************************/
    int _pwmPin;  //PWM pin for the motor object
    int _dirPin;  //DIR pin for the motor object
    bool _dirVal; //default direction value

  /**********************************************************************************************************************/

  public:

  //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  constructor for class Motor
    Input            :  motor pwm and dir pin with the dir value
    Output           :  ~none~
    Logic            :  will pass on the motor pwm and dir pin with dir value when constructor is called 
    Example Call     :  Motor(motorPwmPin, motorDirPin, motorDirValue)
    */
    
    Motor() {}
    Motor(int motorPwmPin, int motorDirPin, bool motorDirValue) {
      _pwmPin = motorPwmPin;
      _dirPin = motorDirPin;
      _dirVal = motorDirValue;
      pinMode(_pwmPin, OUTPUT);
      pinMode(_dirPin, OUTPUT);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  setPwm
    Input            :  motor PWM value
    Output           :  ~none~
    Logic            :  Wll set PWM value of the motor ie analogWrite the value stored in the variable to respective pin
    Example Call     :  setPwm(motorPwmValue)
    */

    void setPwm(int motorPwmValue) {
      analogWrite(_pwmPin, motorPwmValue);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  setDir
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  Wll set Dir value of the motor ie digitalWrite the value stored in the variable to respective pin
                        variable has HIGH or LOW as value only
    Example Call     :  setDir()
    */
    
    void setDir() {
      digitalWrite(_dirPin, _dirVal);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------

   /*
    Function Name    :  dirChange
    Input            :  ~none~
    Output           :  ~none~
    Logic            :  Wll switch Dir value of the motor respective to setDir
    Example Call     :  dirChange()
    */
    
    void dirChange() {
      digitalWrite(_dirPin, !_dirVal);
    }

     //-------------------------------------------------------------------------------------------------------------------------------------------
};

#endif
