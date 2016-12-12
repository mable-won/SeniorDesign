//pinout of LEDs
#define RLED 8
#define YLED 9
#define GLED 10


//status bits
bool charged = 0;
bool needsCharging = 0;


void setup() {
  Serial.begin(57600);
  while (!Serial);
  Serial.println("Voltage");
  //turn off LEDs
 /* digitalWrite(RLED,LOW);
  digitalWrite(YLED,LOW);
  digitalWrite(GLED,LOW);
*/}


float readVoltage() {
  float sum1 = 0, sum2 = 0;
  //Read analog input
  for (int i = 0; i < 200; i++) {
    sum1 += (float)analogRead(A2) / 1023;
    sum2 += (float)analogRead(A0) / 1023;
  } float val1 = sum1 * 3.3 / 200;
  float val2 = sum2 * 3.3 / 200;
  Serial.println(val1-val2);
  //delay(500);
  return (val1-val2);
}


void loop() {
  float voltage = readVoltage();
/*  if (voltage <= 1 && !digitalRead(RLED)) {
    digitalWrite(YLED,LOW);
    digitalWrite(RLED,HIGH);
    needsCharging = 1;
  } else if (voltage > 1 && voltage < 3.3 && !digitalRead(YLED)) {
    digitalWrite(RLED,LOW);
    digitalWrite(GLED,LOW);
    digitalWrite(YLED,HIGH);
    needsCharging = 0;
    charged = 0;
  } else if (voltage >= 3.3 && !digitalRead(GLED)) {
    digitalWrite(YLED,LOW);
    digitalWrite(GLED,HIGH);
    charged = 1;
  }
*/  delay(500);
}

