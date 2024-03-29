# GPS-Speed-and-Acceleration-Measurement-with-Kalman-Filter-Integration
 GPS-based system for accurately measuring speed and acceleration, applying Kalman filtering techniques for enhanced precision. The system further includes a user-friendly OLED display for real-time speed visualization.


### Description:

GPS Speedometer project repository: This open-source project aims to provide a comprehensive GPS-based system for accurately measuring speed and acceleration, applying Kalman filtering techniques for enhanced precision. The system further includes a 1.5 inch OLED display for real-time speed visualization and a web interface for remote monitoring.<b>This project was implemented to measure vehicle telemetry for Team Firebolt Racing in mBAJA 2024 (SAEINDIA).</b>

### Hardware used:
<ol>
 <li>ublox NEO-6M</li>
 <li><a href="https://robu.in/product/gps-glonass-gnss-antenna-for-raspberry-pi-hat-and-arduino-shield-with-3-meter-cable/?gad_source=1&gclid=CjwKCAiA9ourBhAVEiwA3L5RFgNR95wsRVnACjbRkYZ2RfMn0eCl59X-f6alDHBtyXvHmg76V53xPRoCONcQAvD_BwE">GPS antenna (for better performance)</a></li>
 <li>BNO085 9DoF (accelerometer, gyroscope, magnetometer)</li>
 <li>NodeMCU v1.0</li>
 <li>1.5 inch OLED display by Waveshare [SSD1327]</li>
</ol>

## Features:
## 1. GPS-Based Speed and Acceleration Measurement
Utilizing GPS data, the system accurately calculates and measures real-time speed and acceleration. The integration of GPS technology ensures reliable and precise location-based speed readings.

## 2. Kalman Filter Implementation
To enhance the accuracy of speed and acceleration measurements, a Kalman filter is applied. The Kalman filter optimally combines GPS data with accelerometer readings, resulting in more reliable and smoother speed values, particularly in scenarios with signal noise or intermittent GPS connectivity.

## 3. OLED Display
The project includes an OLED display to provide a convenient and intuitive interface for users to monitor their speed in real time. The display showcases clear and easily readable speed information, making it suitable for various applications, such as automotive or fitness devices.

## 4. Webpage Display
For remote monitoring and data visualization, the system features a web interface. Users can access a webpage that displays real-time speed information, allowing for convenient monitoring without the need for physical interaction with the OLED display. The web interface supports multiple devices, providing a flexible and accessible solution.

## Getting Started:
1. Clone the repository to your local machine.
2. Install the required dependencies mentioned in the documentation.
3. Make the required changes in configuration to GPS module using U - Center software.
4. Connect the GPS module, BNO085 and OLED display to NodeMCU to set-up. 
5. Upload the provided code to your microcontroller.
6. Access the web interface to monitor real-time speed remotely.

### Port Map
<ul>
  <li>OLED SCREEN</li>
   <ol>
    <li>VCC -> 3V</li>
    <li>GND -> GND</li>
    <li>DIN -> D7</li>
    <li>CLK -> D5</li>
    <li>CS -> D8</li>
    <li>DC -> D0</li>
    <li>RST -> D3</li>
   </ol>

  <li>BNO085</li>
   <ol>
    <li>VIN -> 3V</li>
    <li>GND -> GND</li>
    <li>SCL -> D1</li>
    <li>SDA -> D2</li>
    <li>P0 -> GND</li>
    <li>P1 -> GND</li>
   </ol>

   <li>GPS</li>
   <ol>
    <li>VCC -> 3V</li>
    <li>GND -> GND</li>
    <li>RX -> D4</li>
    <li>TX -> D6</li>
   </ol>
</ul>

## Contribution Guidelines:
Welcome for contributions to enhance and expand the capabilities of this GPS speedometer system. Feel free to submit bug reports, feature requests, or pull requests to help improve the project.

Thank you for choosing our GPS Speedometer project. We hope it proves to be a valuable tool for your speed measurement and monitoring needs!

## References
<ul>
 <li>
  <a href = "https://youtu.be/TwhCX0c8Xe0?si=p0fIQ2RGdnS_CM2-">GPS Configuration Reference</a>
 </li>
 <li>
  <a href="https://github.com/olikraus/u8g2.git">Library for OLED display</a>
 </li>
 <li>
  <a href = "https://github.com/adafruit/Adafruit_BNO08x.git">Library for BNO085</a>
 </li>
 <li>
  <a href="https://youtu.be/6M6wSLD-8M8?si=LIJLbGDBOQZDQuQw">Kalman Filter Basics</a>
 </li>
 <li>
  <a href="https://github.com/morrissinger/ESP8266-Websocket.git">ESP8266 Web Sockets</a>
 </li>
</ul>
