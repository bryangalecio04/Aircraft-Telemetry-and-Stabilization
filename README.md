# Aircraft Telemetry and Stabilization
In response to a project emphasizing creativity and sensor integration assigned to us by our professor, my peers, Kevin Galarraga and Nick Colaco, and I designed and implemented a scaled-down cardboard aircraft prototype. The system acquires and displays real-time flight and environmental telemetry on an LCD interface while simultaneously transmitting the data to a remote server over Wi-Fi. It incorporates multiple sensing modules for distance, temperature, humidity, liquid level, acoustic events, and secure RFID-based access control, forming a comprehensive sensor network for aircraft monitoring.

The aircraft utilizes a 3-axis accelerometer and a 3-axis gyroscope module to determine orientation and calculate yaw, pitch, and roll. Control surfaces, including flaps, rudders, and elevators, are actuated by servomotors in response to IMU feedback. A stabilization algorithm fuses sensor data and generates corrective control signals, enabling real-time adjustment of the control surfaces to maintain steady flight autonomously.

## Parts List
- **ATmega2560 Arduino Mega Board**  
- **RFID-RC522 Module**  
- **GY-521 MPU-6050 (Accelerometer & Gyroscope)**  
- **HC-SR04 Ultrasonic Sensor**  
- **DHT11 Temperature & Humidity Sensor**  
- **MAX4466 Microphone Amplifier**  
- **Water Level Sensor**  
- **Laser Module**  
- **4× SG90 DC Servo Motors**  
- **16×2 LCD with I2C Interface**  
- **DFPlayer Mini MP3 Player**  
- **3 W, 8 Ω Mini Speaker**  
- **1K Ω Resistors**  
- **9V Battery**  
- **5V 2A Power Source**  
- **Switch Button**