#include <SoftwareSerial.h>
#define D0 16
#define D1 5
#define D2 4
#define D4 2
#define D6 12
SoftwareSerial gpsSerial(/*GPS TX = */ D6, /*GPS RX = */D4);  // TX, RX pins for GPS connection

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.flush();
  gpsSerial.flush();
}
void loop() {
  if (gpsSerial.available()) {
    Serial.write(gpsSerial.read());
  }
  if (Serial.available()) {
    gpsSerial.write(Serial.read());
  }
}

