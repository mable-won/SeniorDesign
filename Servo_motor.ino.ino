#include <Servo.h>
Servo servoMain; 

void setup()
{
 servoMain.attach(10);
 //Serial.begin(57600);
  
  }

void loop(){
  digitalWrite(2,HIGH);
  closeServo();


  }

  void openServo(){
    servoMain.write(90);
    }

      void closeServo(){
    servoMain.write(30);
    }
