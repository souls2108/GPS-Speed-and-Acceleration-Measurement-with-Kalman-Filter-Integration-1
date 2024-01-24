#include <U8g2lib.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <quaternion_type.h>
// #include <Wire.h>
// #include  <SPI.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
//-------------------VARIABLES YOU MAY NEED TO CHANGE------------------------
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D6 12

#define IMU_noise 1
const char *ssid = "NNN";
const char *password = "00000000";
#define IP_display_delay 700
//to make changes to oled display check layout init function
//---------------------------------------------------------------------------

//------INITIALISATION OF HARDWARE OBJECTS-----------------------------------
SoftwareSerial GPS(/*GPS TX = */ D6, /*GPS RX = */D4);  // TX, RX pins for GPS connection
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
/*
CLK GPIO 14 D5
MOSI GPIO 13  D7
CS  15  D8
RST D3
DC  D0
*/
U8G2_SSD1327_WS_128X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 15, /* dc=*/ 16, /* reset=*/ 0);
typedef u8g2_uint_t u8g_uint_t; 

//------ GPS related functions ----------------------------------------------- 
// DATA PACKET RECIEVED FROM UBLOX GPS MODULE
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
struct NAV_VELNED {
  unsigned char cls;
  unsigned char id;
  unsigned short len;

  unsigned long int iTOW;
  long int velN;
  long int velE;
  long int velD;
  unsigned long int speed;
  unsigned long int gSpeed;
  long int heading;
  unsigned long int sAcc;
  unsigned long int cAcc;
};
NAV_VELNED velned;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_VELNED); i++) {
    CK[0] += ((unsigned char*)(&velned))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_VELNED);

  while ( GPS.available() ) {
    byte c = GPS.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&velned))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

//------------ IMU functions ------------------------------------------
struct NED {
  float N;
  float E;
  float D;
  unsigned long log_time;
};
unsigned long last_gps_log_time = 0;
float prev_process_error = 20;
NED accNED;
NED filtered_speedNED;
quat_t quat;
#define low_val_offset 0.07 //ignore small accelerations when stationary


void setReports(void) {
  displayMessage("Setting desired","reports");
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    displayMessage("Could not enable","linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR) ) {
    displayMessage("Could not enable geomagnetic","rotation vector");
  }
  // if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
  //   Serial.println("Could not enable SH2_ARVR_STABILIZED_RV");
  // }
}

int processIMU() {  
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return 0;
  }
  switch (sensorValue.sensorId) {
    case SH2_LINEAR_ACCELERATION: {
      quat_t qa = { 0, sensorValue.un.linearAcceleration.x,
      sensorValue.un.linearAcceleration.y,
      sensorValue.un.linearAcceleration.z};
      quat = quat.norm();
      quat_t res = quat * qa * quat.conj();
      accNED.N = res.v.y;
      accNED.E = res.v.x;
      accNED.D = -res.v.z;
      if(accNED.N < low_val_offset && accNED.N > -low_val_offset) {
        accNED.N = 0;
      }
      if(accNED.E < low_val_offset && accNED.E > -low_val_offset) {
        accNED.E = 0;
      }
      if(accNED.D < low_val_offset && accNED.D > -low_val_offset) {
        accNED.D = 0;
      }
      accNED.log_time = micros();
      return 1;
    }

    case SH2_GEOMAGNETIC_ROTATION_VECTOR: {
      quat = { sensorValue.un.geoMagRotationVector.real, 
      sensorValue.un.geoMagRotationVector.i,
      sensorValue.un.geoMagRotationVector.j,
      sensorValue.un.geoMagRotationVector.k};
      return 2;
    }
  } 
  return 0;
}
struct euler_t {
  float yaw;
  float pitch;
  float roll;
};
euler_t ypr;
void quaternionToEuler(float qr, float qi, float qj, float qk) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    // conversion to degrees 
    ypr.yaw *= RAD_TO_DEG;
    ypr.pitch *= RAD_TO_DEG;
    ypr.roll *= RAD_TO_DEG;
}

//-------CORE PROCESS COLLECTING AVAILABLE DATA (KALMAN FILTER APPLIED HERE)--------------------------------------------

void process() {
  if (processGPS()) {
    unsigned long now = micros();
    float time_gps = (now - last_gps_log_time) / 1000000.0;
    last_gps_log_time = now;
    /*BNO085 datasheet mentions error in linear acc - 0.35 m/s², but 1 m/s² noise worked for me*/
    prev_process_error += sq(IMU_noise * time_gps); 
    float kalmanGain = prev_process_error / ( prev_process_error + sq(velned.sAcc / 100.0));
    filtered_speedNED.N = filtered_speedNED.N + kalmanGain * ((velned.velN / 100.0) - filtered_speedNED.N);
    filtered_speedNED.E = filtered_speedNED.E + kalmanGain * ((velned.velE / 100.0) - filtered_speedNED.E);
    filtered_speedNED.D = filtered_speedNED.D + kalmanGain * ((velned.velD / 100.0) - filtered_speedNED.D);
    filtered_speedNED.log_time = now;
    prev_process_error = (1 - kalmanGain) * prev_process_error; //Update error covariance


    Serial.print("spd GPS:0,");
    Serial.print("N:");
    Serial.print(filtered_speedNED.N);
    Serial.print(",E:");
    Serial.print(filtered_speedNED.E);
    Serial.print(",D:");
    Serial.print(filtered_speedNED.D);
    Serial.print(",kg:");
    Serial.print(kalmanGain);
    Serial.print(",log_time:");
    Serial.print(filtered_speedNED.log_time);
    Serial.println();


    // Serial.print("GPS:0,");  
    Serial.print("velnedN:");
    Serial.print(velned.velN / 100.0);
    Serial.print(",velnedE:");
    Serial.print(velned.velE / 100.0);
    Serial.print(",velnedD:");
    Serial.print(velned.velD / 100.0);
    Serial.print(",spdAcc:");
    Serial.print(velned.sAcc / 100.0);
    Serial.println();

    return;
  }
  switch (processIMU()) {
    case 1: /* Integrate if new acceleration reading is recieved*/ {
      float time_acc = (accNED.log_time - filtered_speedNED.log_time)/1000000.0; //conversion micro secs to secs without loss float precision
      filtered_speedNED.N += accNED.N * time_acc;
      filtered_speedNED.E += accNED.E * time_acc;
      filtered_speedNED.D += accNED.D * time_acc;
      filtered_speedNED.log_time = accNED.log_time;
      
      Serial.print("spd acc:0,");
      Serial.print("N:");
      Serial.print(filtered_speedNED.N);
      Serial.print(",E:");
      Serial.print(filtered_speedNED.E);
      Serial.print(",D:");
      Serial.print(filtered_speedNED.D);
      // Serial.print(",log_time:");
      // Serial.print(filtered_speedNED.log_time);
      Serial.println();

      
      Serial.print("Type:4,");
      Serial.print("N:");
      Serial.print(accNED.N);
      Serial.print(",E:");
      Serial.print(accNED.E);
      Serial.print(",D:");
      Serial.print(accNED.D);
      // Serial.print(",log_time:");
      // Serial.print(accNED.log_time);
      Serial.println();
      break;
    }
    case 2:
      /*Update ypr subract from ypr offset, yaw value invalid*/
      quaternionToEuler(quat.w, quat.v.x, quat.v.y, quat.v.z);

      // Serial.print("Type:2,");
      // Serial.print("yaw:");
      // Serial.print(ypr.yaw);
      // Serial.print(",pitch:");
      // Serial.print(ypr.pitch);
      // Serial.print(",roll:");
      // Serial.print(ypr.roll);
      // Serial.println();
      break;

    case 0:
      //Serial.println(0);
      break;
  }
}

//initialisation of variables for kalman filter
void varDefaultInit() {
      filtered_speedNED.N = 0; accNED.N = 0;
      filtered_speedNED.E = 0; accNED.E = 0;
      filtered_speedNED.D = 0; accNED.D = 0;
      filtered_speedNED.log_time = 0; accNED.log_time = 0;
      velned.sAcc = 2000;
}

/*      ------------    SCREEN HANDLER    ------------      */
// void displayScreen(){
//   u8g2.clearBuffer();
//   u8g2.setFontMode(0);
//   u8g2.setFont(u8g2_font_helvR14_tf);

//   // u8g2.setCursor(10, 10);
//   // u8g2.print("Screen Test");

//   u8g2.setCursor(10 , 30);
//   u8g2.print(sqrt(sq(filtered_speedNED.N) + sq(filtered_speedNED.E) + sq(filtered_speedNED.D)));

//   u8g2.setCursor(u8g2.getDisplayWidth()/2 , 30);
//   u8g2.print(sqrt(sq(accNED.N) + sq(accNED.E) + sq(accNED.D)));

//   u8g2.setFont(u8g2_font_5x7_mf);
//   u8g2.setCursor(0, u8g2.getDisplayHeight() - 10);
//   u8g2.print(ypr.yaw);
//   u8g2.print(" ");
//   u8g2.print(ypr.pitch);
//   u8g2.print(" ");
//   u8g2.print(ypr.roll);

//   u8g2.sendBuffer();
//   delay(1);
// }

struct Layout {
  const uint8_t* font1;
  const uint8_t* font2;
  const uint8_t* font3;
  const uint8_t* font4;

  u8g_uint_t spd_x;
  u8g_uint_t spd_y;

  u8g_uint_t dial_x;
  u8g_uint_t dial_y;
  u8g_uint_t rad_in;
  u8g_uint_t rad_out;
  unsigned int aSpace;

  u8g_uint_t acc_x;
  u8g_uint_t acc_y;
  u8g_uint_t bar_width;

  u8g_uint_t kmph_txt_x;
  u8g_uint_t kmph_txt_y;
};
Layout layout;

void layoutinit() {
  // fonts
  layout.font1 =  u8g2_font_logisoso54_tf;
  layout.font2 =  u8g2_font_helvR24_tf;
  layout.font3 =  u8g2_font_courR12_tf;
  layout.font4 =  u8g2_font_courR12_tf;

  //center dial
  u8g_uint_t width = 4;
  layout.dial_x = u8g2.getDisplayWidth() / 2;
  layout.rad_out = u8g2.getDisplayWidth() / 2;
  layout.rad_in = layout.rad_out - width;
  layout.dial_y = u8g2.getDisplayHeight() - (cos(radians(45)) * layout.rad_out);
  layout.aSpace = 30;
  
  //top bar
  layout.bar_width = 8;

  //speed txt
  layout.spd_x = u8g2.getDisplayWidth() / 2;
  layout.spd_y = u8g2.getDisplayHeight() / 2;

  //acc txt
  layout.acc_x = u8g2.getDisplayWidth() - 30;
  layout.acc_y = u8g2.getDisplayHeight() - 30;

  //kmph txt
  layout.kmph_txt_x = u8g2.getDisplayWidth() / 2 - (layout.rad_in * cos(radians(45))) + /*padding */ 5;
  layout.kmph_txt_y = u8g2.getDisplayHeight() - width / 2 - /*padding*/ 5;

}

void displayScreen() {
  u8g2.clearBuffer();

  //Speed txt
  unsigned int speedkmph = int(sqrt(sq(filtered_speedNED.N) + sq(filtered_speedNED.E) + sq(filtered_speedNED.D)) * 3.6);
  u8g2.setFont(layout.font1);
  String buf = String(speedkmph);
  u8g2.setCursor(layout.spd_x - (u8g2.getStrWidth(buf.c_str())) / 2, layout.spd_y);
  u8g2.setFontPosCenter();
  u8g2.print(speedkmph);
  u8g2.setFontMode(0);

  //Drawing dots of arc
  for(int i = 225; i >= 0; i = i - layout.aSpace) {
    u8g2.drawBox(layout.dial_x + layout.rad_out * cos(radians(i)), layout.dial_y - layout.rad_out * sin(radians(i)), 2, 2);
  }
  //Drawing dial arc
  u8g2_draw_arc(layout.dial_x, layout.dial_y, layout.rad_in, layout.rad_out, 225 - ((speedkmph * 225) / /*max spd*/ 100 ), 225);

  //Acc txt
  unsigned int acc = int(sqrt(sq(accNED.N) + sq(accNED.E) /*+ sq(accNED.D)*/)); //Maybe Down causes noise ??
  u8g2.setFont(layout.font2);
  u8g2.setCursor(layout.acc_x, layout.acc_y);
  u8g2.setFontPosCenter();
  u8g2.print(acc);  
  //Draw acc bar
  u8g2.drawBox(0, 0, (u8g2.getDisplayWidth() * acc) / /*max_acc*/ 20, /*bar width - */ layout.bar_width);

  //Print kmph
  u8g2.setFont(layout.font3);
  u8g2.setCursor(layout.kmph_txt_x, layout.kmph_txt_y);
  u8g2.setFontPosCenter();
  u8g2.print("Kmph");



  u8g2.sendBuffer();
}
void displayMessage(String msg){
  displayMessage(msg, "");
}
void displayMessage(String msg, String msg_small) {
  Serial.println(msg);
  u8g2.clearBuffer();

  u8g2.setFont(layout.font3);
  u8g2.setFontPosCenter();
  u8g2.setCursor(2, 15);
  u8g2.print("[Info]");
  u8g2.setCursor(1, 50);
  u8g2.print(msg.c_str());
  u8g2.setCursor(1, 70);
  u8g2.print(msg_small.c_str());

  u8g2.sendBuffer();
  // delay(1000);
}

//-------------WEBPAGE STUFF------------------------------
void displayWebpage(){
  unsigned int speedkmph = int(sqrt(sq(filtered_speedNED.N) + sq(filtered_speedNED.E) + sq(filtered_speedNED.D)) * 3.6);
  unsigned int acc = int(sqrt(sq(accNED.N) + sq(accNED.E) /*+ sq(accNED.D)*/)); //Maybe Down causes noise ??
  server.handleClient();
  webSocket.loop();

  // String payload = "Speed : " + String(speedkmph) + " kmph" + "  " + 
  //               "Acc : " + String(acc) + " m/s²";
  int sensorValue = speedkmph;
  String payload = String(sensorValue);
  webSocket.broadcastTXT(payload);

}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Sensor Data</h1>";
  html += "<p id='speedkmph'>Loading...</p>";
  html += "<script>var socket1 = new WebSocket('ws://' + window.location.hostname + ':81/');";
  html += "socket1.onmessage = function(event) {document.getElementById('speedkmph').innerHTML = 'Speed(kmph): ' + event.data;};";
  html += "</script>";

  html += "<p id='acc'>Loading...</p>";
  html += "<script>var socket = new WebSocket('ws://' + window.location.hostname + ':81/');";
  html += "socket.onmessage = function(event) {document.getElementById('acc').innerHTML = 'Acceleration(m/s²): ' + event.data;};";
  html += "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}
//---------------------------------------------------------

// void debug_gps() {  
//   Serial.print(velned.cls);
//   Serial.print(","); Serial.print(velned.id);
//   Serial.print(","); Serial.print(velned.len);
//   Serial.print(",");Serial.print(velned.iTOW);
//   Serial.print(","); Serial.print(velned.velN);
//   Serial.print(","); Serial.print(velned.velE);
//   Serial.print(","); Serial.print(velned.velD);
//   Serial.print(","); Serial.print(velned.speed);
//   Serial.print(","); Serial.print(velned.gSpeed);
//   Serial.print(","); Serial.print(velned.heading);
//   Serial.print(","); Serial.print(velned.sAcc);
//   Serial.print(","); Serial.print(velned.cAcc);
//   Serial.println();

//   u8g2.clearBuffer();
//   u8g2.setFontMode(0);
//   u8g2.setFont(u8g2_font_helvR14_tf);
//   u8g2.setCursor(0,20);

//   u8g2.print(velned.velN); u8g2.print(","); u8g2.print(velned.velE); u8g2.print(","); u8g2.print(velned.velD);
//   u8g2.setCursor(0, 40);
//   u8g2.print(velned.speed); u8g2.print(","); u8g2.print(velned.sAcc);
//   u8g2.sendBuffer();
// }


//---------NodeMCU setup and hardware checkup--------------------------
void setup() {
  u8g2.begin();
  // u8g2.enableUTF8Print();
  layoutinit();

  Serial.begin(9600);
  Serial.flush();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, handleRoot);
  webSocket.begin();
  server.begin();
  displayMessage("IP address:", WiFi.softAPIP().toString());
  delay(IP_display_delay);
  displayMessage("Server active");
  delay(IP_display_delay);

  GPS.begin(9600);
  GPS.flush();

  if (!bno08x.begin_I2C()) {
    displayMessage("Failed to find","BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  displayMessage("BNO08x Found!");
  setReports(); //SETTING imu BNO085 mode

  displayMessage("Checking sensor status.");
  varDefaultInit();
  checkSensorStatus();
}

void checkSensorStatus() {
  do {
    bno08x.getSensorEvent(&sensorValue);
    displayMessage("BNO085 : " + String(sensorValue.status));
    delay(1);
  } while(sensorValue.status != 3);
  do {
      processGPS();
      displayMessage("GPS : " + String(velned.sAcc / 100));
      delay(1000);
  } while(2000 == velned.sAcc);
}

void loop() {
  // Serial.println(".");
  process();
  // if(processGPS())
  //   debug_gps();
  displayScreen();
  displayWebpage();
}


static const float pi_2 = 1.57079632679489661923;
static const float pi_4 = 0.78539816339744830962;
static void u8g2_draw_arc(u8g2_uint_t x0, u8g2_uint_t y0, u8g2_uint_t rad_in, u8g2_uint_t rad_out, u8g2_uint_t angle_start, u8g2_uint_t angle_end)
{
  // Declare variables
  u8g2_long_t x, y, d, r, as, ae, cnt, num_pts;

  // Manage angle inputs
  uint8_t inverted = (angle_start > angle_end);
  as = inverted ? angle_end : angle_start;
  ae = inverted ? angle_start : angle_end;

  // Trace each arc radius with the Andres circle algorithm
  for(r = rad_in; r <= rad_out; r++)
  {
    x = 0;
    y = r;
    d = r - 1;
    //num_pts = atan2f_func ? 100 : (r * 8 / 10); // if no atan2f() function is provided, we make a low cost approximation of the number of pixels drawn for a 1/8th circle of radius r
    num_pts =100;
    // Process each pixel of a 1/8th circle of radius r
    while (y >= x)
    {
      // If atan2f() function is provided, get the percentage of 1/8th circle drawn, otherwise count the drawn pixels
      //cnt = atan2f_func ? ((pi_2 - atan2f_func(y, x)) * 100 / pi_4) : (cnt + 1);
      cnt = ((pi_2 - atan2f(y, x)) * 100 / pi_4);
      // Fill the pixels of the 8 sections of the circle, but only on the arc defined by the angles (start and end)
      if((cnt > num_pts * as / 45 && cnt <= num_pts * ae / 45) ^ inverted) u8g2.drawPixel(x0 + y, y0 - x);
      if((cnt > num_pts * (90 - ae) / 45 && cnt <= num_pts * (90 - as) / 45) ^ inverted) u8g2.drawPixel(x0 + x, y0 - y);
      if((cnt > num_pts * (as - 90) / 45 && cnt <= num_pts * (ae - 90) / 45) ^ inverted) u8g2.drawPixel(x0 - x, y0 - y);
      if((cnt > num_pts * (180 - ae) / 45 && cnt <= num_pts * (180 - as) / 45) ^ inverted) u8g2.drawPixel(x0 - y, y0 - x);
      if((cnt > num_pts * (as - 180) / 45 && cnt <= num_pts * (ae - 180) / 45) ^ inverted) u8g2.drawPixel(x0 - y, y0 + x);
      if((cnt > num_pts * (270 - ae) / 45 && cnt <= num_pts * (270 - as) / 45) ^ inverted) u8g2.drawPixel(x0 - x, y0 + y);
      if((cnt > num_pts * (as - 270) / 45 && cnt <= num_pts * (ae - 270) / 45) ^ inverted) u8g2.drawPixel(x0 + x, y0 + y);
      if((cnt > num_pts * (360 - ae) / 45 && cnt <= num_pts * (360 - as) / 45) ^ inverted) u8g2.drawPixel(x0 + y, y0 + x);

      // Run Andres circle algorithm to get to the next pixel
      if (d >= 2 * x)
      {
        d = d - 2 * x - 1;
        x = x + 1;
      } else if (d < 2 * (r - y))
      {
        d = d + 2 * y - 1;
        y = y - 1;
      } else
      {
        d = d + 2 * (y - x - 1);
        y = y - 1;
        x = x + 1;
      }
    }
  }
}
