#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(10, 11);  // RX, TX pins for GPS connection

void setup() {
  Serial.begin(57600);
  gpsSerial.begin(57600);
}
void loop() {
  if (gpsSerial.available()) {
    Serial.write(gpsSerial.read());
  }
  if (Serial.available()) {
    gpsSerial.write(Serial.read());
  }
}

