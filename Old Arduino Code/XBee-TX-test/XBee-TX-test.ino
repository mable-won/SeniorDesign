#include <XBee.h>

XBee xbee = XBee();

//unsigned long start = millis();

//Enter message to be transmitted.
uint8_t payload[] = {'H','e','l','l','o'};

// with Series 1 you can use either 16-bit or 64-bit addressing
// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload));

// 64-bit addressing: This is the SH + SL address of remote XBee
//XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4008b490);
// unless you have MY on the receiving radio set to FFFF, this will be received as a RX16 packet
//Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));

TxStatusResponse txStatus = TxStatusResponse();

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
}

void loop() {
   
   // start transmitting after a startup delay.  Note: this will rollover to 0 eventually so not best way to handle
   // if (millis() - start > 15000) {
      // set payload here
      xbee.send(tx);
    //}
    // after sending a tx request, we expect a status response
    // wait up to 5 seconds for the status response
    if (xbee.readPacket(5000)) {
        // got a response!
        // should be a znet tx status              
      if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
         xbee.getResponse().getTxStatusResponse(txStatus);
        
         // get the delivery status, the fifth byte
           if (txStatus.getStatus() == SUCCESS) {
              Serial.println("Message delivered successfully.");
           } else {
              Serial.println("ERROR: Message not received.");
           }
        }      
    } else if (xbee.getResponse().isError()) {
      Serial.println("ERROR reading package.");
    } else {
      Serial.println("ERROR: Response Timeout.");
    }
    delay(1000);
}
