/* Filename                 : MPU6050.h
 * Name of Class            : MPU6050
 * Name of Methods          : mpuread()
 * Name of Member Variables : 
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

class MPU6050{
  int8_t m; //variable to store the directly transmitted mpu value
  int yawOld; //variable to store the mapped mpu value
  
  public:
  MPU6050(){
    
  }

  //-------------------------------------------------------------------------------------------------------------------------------------------

/*
    Function Name    :  mpuread()
    Input            :  ~none~
    Output           :  mapped yaw coming from the mpuboard ie mpuOld 
    Logic            :  will serial read data coming from mpuboard and map it to -180 to 179 for proper rotation.
    Example Call     :  mpuRead()
    */

  int mpuRead(){
    if(Serial2.available()){
      m = Serial2.read();
    }
    yawOld = map(m, -128, 127, 0, 360);
    return yawOld;
  }

  //-------------------------------------------------------------------------------------------------------------------------------------------
  
};

#endif
