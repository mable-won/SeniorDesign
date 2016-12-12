//Motorcontoller.ono


#ifndef __AVR_ATmega32U4__
#define __AVR_ATmega32U4__
#endif


#define MODE_PIN 7


#define M1PWM 3
#define M1PH 4


#define M2PWM 5
#define M2PH 6


void setup()
{
  Serial.begin(57600);
  Serial1.begin(57600);
  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);


  // Update these if the directions of the wheels are wrong
  digitalWrite(M1PH, HIGH);
  digitalWrite(M2PH, HIGH);
}


// Read buffer, head is 'C', tail is 'M'
int bufferSize = 4; //size is 2+2*number of cars
byte readBuffer[4];


// Vehicle ID
int carID = 1;


// Calibration parameters. For all vehicles, at leftMax + rightMax they
// should go roughly along a straight line forward at roughly the same speed
int leftMax = 200;
int rightMax = 200;


// Previous commands
byte hasPreviousCommand = 0;
byte previousLeft = 0;
byte previousRight = 0;


// Run the two motors at different thrusts, negative thrusts makes the wheel
// go backward
void runMotors(int rightThrust, int leftThrust) {
  // Left motor
  if (rightThrust > 0) {
    digitalWrite(M1PH, HIGH);
  }
  else {
    digitalWrite(M1PH, LOW);
  }


  // Right motor
  if (leftThrust > 0) {
    digitalWrite(M2PH, HIGH);
  }
  else {
    digitalWrite(M2PH, LOW);
  }


  // Write the motor speeds
  analogWrite(M1PWM, abs(rightThrust)*leftMax / 128);
  analogWrite(M2PWM, abs(leftThrust)*rightMax / 128);
}




void processCommand(byte readBuffer[]) {
  // Parse the command
  byte lc = readBuffer[carID * 2 - 1];
  byte rc = readBuffer[carID * 2];


  // Parse and send command to motor
  if (hasPreviousCommand == 0 ||
      (hasPreviousCommand == 1 &&
       (previousLeft != lc || previousRight != rc)
      ))
  {
    int rightThrust = (rc & 0x7F) * ((rc & 0x80) == 0x80 ? -1 : 1);
    int leftThrust = (lc & 0x7F) * ((lc & 0x80) == 0x80 ? -1 : 1);
    runMotors(rightThrust, leftThrust);


    // Save command
    previousRight = rc;
    previousLeft = lc;
    hasPreviousCommand = 1;
  }
}


void moveStraight(float distance /*in cm*/ , float thrust /* fraction between 0 and 1 */) {
  readBuffer[0] = 'C';
  readBuffer[bufferSize-1] = 'M';
  if (distance > 0) {
    readBuffer[2 * carID - 1] = int(127*thrust);
    readBuffer[2 * carID] = int(127*thrust);
  } else if (distance < 0) {
    readBuffer[2 * carID - 1] = int(127*thrust)|0x80;
    readBuffer[2 * carID] = int(127*thrust)|0x80;
  } else {
    Serial.println("Distance cannot be 0.");
    return;
  }
  processCommand(readBuffer);
  delay(abs(distance) / (20.5*thrust) * 1000);
}


void rotateVehicle(float angle, float thrust /* fraction between 0 and 1 */) {
  //Angle in degrees from -180 to 180 with reference to the y-axis (ex. an angle of 90 goes right)
  readBuffer[0] = 'C';
  readBuffer[bufferSize] = 'M';
  if (angle > 0) {
    readBuffer[2 * carID - 1] = int(127*thrust);
    readBuffer[2 * carID] = int(127*thrust)|0x80;
  } else if (angle < 0) {
    readBuffer[2 * carID - 1] = int(127*thrust)|0x80;
    readBuffer[2 * carID] = int(127*thrust);
    angle = -1 * angle;
  } else {
    Serial.println("Angle cannot be 0.");
    return;
  }
  processCommand(readBuffer);
  
  delay(angle * 0.07505 / (20.5/thrust) * 1000);
  //0.07505 is cm/degree movement for 4.3 cm between wheels
}


void stopVehicle() {
  readBuffer[0] = 'C';
  readBuffer[bufferSize] = 'M';
  readBuffer[2 * carID - 1] = 0x00;
  readBuffer[2 * carID] = 0x00;
  processCommand(readBuffer);
}


void loop() {
  //robot commands
  moveStraight(10,0.8);
  rotateVehicle(90,0.8);
  stopVehicle();
  delay(500);
  //while (1);
}

