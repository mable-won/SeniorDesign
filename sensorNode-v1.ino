/*
  All components of this library are licensed under the BSD 3-Clause
  License.

  Copyright (c) 2015-, Algorithmic Robotics and Control Group @Rutgers
  (http://arc.cs.rutgers.edu). All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.  Redistributions
  in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution. Neither the name of
  Rutgers University nor the names of the contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __AVR_ATmega32U4__
#define __AVR_ATmega32U4__
#endif

#define MODE_PIN 7

#define M1PWM 3
#define M1PH 4

#define M2PWM 5
#define M2PH 6

#include <XBee.h>
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
uint8_t payload[] = {0, 0};
Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

void setup()
{
  //Serial.begin(57600);
  Serial1.begin(57600);
  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  // Update these if the directions of the wheels are wrong
  digitalWrite(M1PH, LOW);
  digitalWrite(M2PH, LOW);
  xbee.setSerial(Serial1);
  //Serial.println("Starting up...");
}

// Delay time
int delayMS = 10;

// Read buffer, head is 'C', tail is 'M'; bufferSize is 2+2*#ofcars
int bufferSize = 18;
byte readBuffer[18];

// Global loop counter
int loopCount = 0;

// Vehicle ID: Change for each vehicle!!
int carID = 1;

// Calibration parameters. For all vehicles, at leftMax + rightMax they
// should go roughly along a straight line forward at roughly the same speed
int leftMax = 200;
int rightMax = 200;

// Previous commands
byte hasPreviousCommand = 0;
byte previousLeft = 0;
byte previousRight = 0;

uint8_t option = 0;
uint8_t data = 0;

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

void resetVehicle() {
  if (loopCount++ < 5) return;
  loopCount = 0;
  // Stop the vehicle
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
  hasPreviousCommand = 0;
  while (Serial1.read() != -1);
}

void sendData(int voltage) {
  // start transmitting after a startup delay.  Note: this will rollover to 0 eventually so not best way to handle
  // if (millis() - start > 15000) {
  // set payload here
  payload[0] = (voltage & 0xFF00) >> 8;
  payload[1] = (voltage & 0x00FF);
  //Serial.println(payload[0]);
  //Serial.println(payload[1]);
  xbee.send(tx);
  //}
  // after sending a tx request, we expect a status response
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(2000)) {
    // got a response!
    // should be a znet tx status
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getStatus() == SUCCESS) {
        //Serial.println("Message delivered successfully.");
      } else {
        //Serial.println("ERROR: Message not received.");
      }
    }
  } else if (xbee.getResponse().isError()) {
    //Serial.println("ERROR reading package.");
  } else {
    //Serial.println("ERROR: Response Timeout.");
  }
}

int readVoltage() {
  float sum1 = 0, sum2 = 0;
  for (int i = 0; i < 200; i++) {
    sum1 += analogRead(A2);
    sum2 += analogRead(A0);
  } int val1 = int(sum1 / 200 + 0.5);
  int val2 = int(sum2 / 200 + 0.5);
  return (val1 - val2);
}

void loop()
{
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    // got something
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      // got a rx packet
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
        xbee.getResponse().getRx16Response(rx16);
        option = rx16.getOption();
        for (int i = 0; i < rx16.getDataLength(); i++) {
          data = rx16.getData(i);
          readBuffer[i] = data;
          //Serial.print(data, HEX); Serial.print(" ");
        }
      } else {
        xbee.getResponse().getRx64Response(rx64);
        option = rx64.getOption();
        // Extract data from package
        for (int i = 0; i < rx64.getDataLength(); i++) {
          data = rx64.getData(i);
          readBuffer[i] = data;
          //Serial.print(data, HEX); Serial.print(" ");
        }
      }
      //Serial.println();
      // Parse command if it seems to be a good one
      if (readBuffer[0] == 'C' && readBuffer[bufferSize - 1] == 'M') {
        loopCount = 0;
        processCommand(readBuffer);
      } else if (readBuffer[0] == 'V' && readBuffer[bufferSize -1] == '?') {
        loopCount = 0;
        int voltage = readVoltage();
        //Serial.println(voltage);
        delay(25 * carID);
        sendData(voltage);
        // Otherwise, reset and flush buffer
      } else {
        resetVehicle();
      }
    }
  } else if (xbee.getResponse().isError()) {
    //Serial.print("Error reading packet.  Error code: ");
    //Serial.println(xbee.getResponse().getErrorCode());
  }
}

