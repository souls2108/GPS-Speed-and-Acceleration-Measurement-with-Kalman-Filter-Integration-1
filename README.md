# GPS-Speed-and-Acceleration-Measurement-with-Kalman-Filter-Integration
 GPS-based system for accurately measuring speed and acceleration, applying Kalman filtering techniques for enhanced precision. The system further includes a user-friendly OLED display for real-time speed visualization.


### Description:

GPS Speedometer project repository: This open-source project aims to provide a comprehensive GPS-based system for accurately measuring speed and acceleration, applying Kalman filtering techniques for enhanced precision. The system further includes a 1.5 inch OLED display for real-time speed visualization and a web interface for remote monitoring.<b>This project was implemented to measure vehicle telemetry for Team Firebolt Racing in mBAJA 2024 (SAEINDIA).</b>

### Hardware used:
<ol>
 <li>ublox NEO-6M</li>
 <li><a href="">GPS antenna (for better performance)</a></li>
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

## Contribution Guidelines:
We welcome contributions from the community to enhance and expand the capabilities of this GPS speedometer system. Feel free to submit bug reports, feature requests, or pull requests to help improve the project.

Thank you for choosing our GPS Speedometer project. We hope it proves to be a valuable tool for your speed measurement and monitoring needs!
