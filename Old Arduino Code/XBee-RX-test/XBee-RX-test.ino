#include <XBee.h>
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
uint8_t option = 0;
uint8_t data = 0;

void setup() {
  // start serial
  Serial.begin(57600);
  Serial1.begin(57600);
  xbee.setSerial(Serial1);

}

void loop() {
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
          Serial.print(data, HEX); Serial.print(" ");
        }
      } else {
        xbee.getResponse().getRx64Response(rx64);
        option = rx64.getOption();
        for (int i = 0; i < rx64.getDataLength(); i++) {
          data = rx64.getData(i);
          Serial.print(data); Serial.print(" ");
        }
      }
      // set dataLed PWM to value of the first byte in the data
      Serial.println();
    }
  } else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");
    //nss.println(xbee.getResponse().getErrorCode());
    // or flash error led
  }
}
