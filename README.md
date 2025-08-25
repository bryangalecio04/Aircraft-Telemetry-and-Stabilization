# Aircraft Telemetry and Stabilization
In response to a project challenge emphasizing creativity and sensor integration by our professor, my peers, Kevin Galarraga and Nick Colaco, and I designed and developed a scaled-down cardboard plane prototype. The system collects and displays real-time flight and environmental data on an LCD dashboard and streams it to a live server over Wi-Fi. It integrates multiple sensors to measure distance, temperature, humidity, water level, sound events, and secure RFID access, providing a comprehensive telemetry system.

The plane uses a 3-axis accelerometer and gyroscope mdodule to measure orientation and calculate yaw, pitch, and roll. Servo-driven flaps, rudders, and elevators respond in real time to maintain stable flight, demonstrating autonomous stabilization through sensor fusion and control logic.  

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