#include <Arduino.h>
#include <ArduinoSTL.h>
#include <DFRobotDFPlayerMini.h>
#include <dht.h>
#include <LiquidCrystal_I2C.h>
#include <map>
#include <MFRC522.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>
#include <SHA256.h>
#include <SPI.h>

// Pin Definitions
#define DHTPIN 9
#define echo 25
#define laser 22
#define micButton 2
#define RST_PIN 3
#define SS_PIN 7
#define trig 24
#define yaw 13
#define pitch 12
const int roll[] = {10,11};
const int waterSensorPin = A0;
const int micPin = A1;

// Device Instances
LiquidCrystal_I2C lcd(0x27, 16, 2); // SDA(A4) SCL(A5)
MFRC522 myRFID(SS_PIN, RST_PIN);
DFRobotDFPlayerMini audioPlayer;
MPU6050 mpu;
dht DHT;
SHA256 sha256;
Servo servoYaw;
Servo servoPitch;
Servo servoRoll1;
Servo servoRoll2;

// Flag Variables
bool DMPReady = false;
bool stabilizing = false;
bool objectDetected = false;
bool raining = false;
bool micActive = false;
bool speakerState = false;

// DMP Buffer Initialization
uint8_t FIFOBuffer[64];

// Strings used to structure the telemetry data that will be transmitted to a display
String message = "Telemetry Data";
String begin_marker = "|";
String delim1 = "||";
String delim2 = "|\\|";
String delim3 = "|\\\|";
String delim4 = "|\\\\|";
String delim5 = "|\\\\\|";
String delim6 = "|\\\\\\|";
String delim7 = "|\\\\\\\|";
String end_marker = "|\\\\\\\\|";

// Variables holding current sensor readings to be transmitted over to a WiFi Board to display on a local web server
long distance_s = 0;
bool raining_s = raining;
float yaw_s = 0;
float pitch_s = 0;
float roll_s = 0;
int temp_s = 0;
int hum_s = 0;

void playAudio(int track) {
  audioPlayer.play(track);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(laser, OUTPUT);
  pinMode(micButton, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  SPI.begin();
  myRFID.PCD_Init();

  servoYaw.attach(yaw);
  servoPitch.attach(pitch);
  servoRoll1.attach(roll[0]);
  servoRoll2.attach(roll[1]);
  servoYaw.write(90);
  servoPitch.write(90);
  servoRoll1.write(90);
  servoRoll2.write(90);

  if (audioPlayer.begin(Serial2)) {
    Serial.println("DFPlayer Mini connection successful");
    audioPlayer.volume(75);
  }
  else {
    Serial.println("DFPlayer Mini connection failed");
  }

  mpu.initialize();
  uint8_t status = mpu.dmpInitialize(); // Initialize the MPU6050 and DMP (status == 0 means successful initialization)

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if(mpu.testConnection() && status == 0) {
    Serial.println("MPU6050 and DMP connection successful");
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);
    mpu.setDMPEnabled(true);
    DMPReady = true;
  }
  else {
    Serial.println("MPU6050 and DMP onnection failed");
  }
  audioPlayer.play(1);
  delay(1000);
  audioPlayer.play(2);
  delay(1000);
  lcd.clear();

  // Hash values generated from Secure Hash Algorithm 256 (SHA-256) for storing RFID UIDs
  const std::map<String, String> hash_values = {
    {"DA48956B275C2B246DD1FAD7AD9ACE26777788841645E40C326453A02C3FE11F", "Kevin"},
    {"12968B977F38420482E20F5DC55D2A9B58E8F22193A5BA3F55221FC054150A04", "Nick"},
    {"F4ACB9A1E208B0D03054F464D32A9CD3D90742DDB32F10A8F7563ED2D1ED02B3", "Bryan"}
  };

  while(1) {
    lcd.setCursor(0,0);
    lcd.print("Scan Access Card");
    if ((!myRFID.PICC_IsNewCardPresent()) || (!myRFID.PICC_ReadCardSerial())) continue;

    String content= "";
    for(byte i = 0; i < myRFID.uid.size; i++) {
      content.concat(String((myRFID.uid.uidByte[i] < 0x10) ? "0" : "") + String(myRFID.uid.uidByte[i], HEX));
    }
    content.toUpperCase();

    uint8_t hash[32];
    sha256.reset();
    sha256.update((const uint8_t*)content.c_str(), content.length());
    sha256.finalize(hash, sizeof(hash));

    String hash_value = "";
    for (byte i = 0; i < sizeof(hash); i++) {
      hash_value.concat(String(hash[i] < 0x10 ? "0" : "") + String(hash[i], HEX));
    }
    hash_value.toUpperCase();

    auto i = hash_values.find(hash_value);
    if(i != hash_values.end()) {
      audioPlayer.play(3);
      Serial.println("Access Granted!");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Access Granted!");
      lcd.setCursor(0,1);
      lcd.print("Welcome " + i->second + "!");
      delay(5000);
      lcd.clear();
      break;
    }
    audioPlayer.play(4);
    Serial.println("Access Denied!");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Access Denied!");
    delay(5000);
    lcd.clear();
    }
}

void stabilize(int &targetRotation, int &lastRotation, Servo &servo, bool roll = false, Servo *servo2 = nullptr) {
  if (targetRotation != lastRotation) {
    servo.write(targetRotation);
    delay(50);
    if (roll && servo2 != nullptr) {
      servo2->write(targetRotation);
      delay(50);
    }
    lastRotation = targetRotation;
    lcd.setCursor(0, 0);
    lcd.print("Plane unstable! ");
    lcd.setCursor(0, 1);
    lcd.print("Stablizing...   ");
  }
}

void aircraftStabilizer() {
  Quaternion q; // [w, x, y, z] quaternion container
  VectorFloat gravity; // [x, y, z] gravity vector
  float ypr[3]; // [yaw, pitch, roll] in radians

  static int lastYaw = 90;
  static int lastPitch = 90;
  static int lastRoll = 90;

  mpu.dmpGetQuaternion(&q, FIFOBuffer); // Extract quaternion from FIFO buffer (orientation data from DMP)
  mpu.dmpGetGravity(&gravity, &q); // Calculate gravity vector from quaternion
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Calculate yaw, pitch, and roll angles in radians from quaternion and gravity vector

  float yawDegree = ypr[0] * 180/M_PI;
  float pitchDegree = ypr[2] * 180/M_PI;
  float rollDegree = ypr[1] * 180/M_PI;
  Serial.println("ypr: " + String(yawDegree) + " " + String(pitchDegree) + " " + String(rollDegree));

  int targetYaw = (yawDegree < -10  || yawDegree > 10) ? (90 + yawDegree) : 90;
  int targetPitch = (pitchDegree < -10  || pitchDegree > 10) ? (90 - pitchDegree) : 90;
  int targetRoll = (rollDegree < -10  || rollDegree > 10) ? (90 + rollDegree) : 90;

  stabilize(targetYaw, lastYaw, servoYaw);
  stabilize(targetPitch, lastPitch, servoPitch);
  stabilize(targetRoll, lastRoll, servoRoll1, true, &servoRoll2);

  stabilizing = !(targetYaw == 90 && targetPitch == 90 && targetRoll == 90);

  yaw_s = yawDegree;
  pitch_s = pitchDegree;
  roll_s = rollDegree;
}

void ultra_sonic_sensor() {
  long duration, distance;

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Read the echoPin, which returns the sound wave travel time in microseconds
  duration = pulseIn(echo, HIGH); // pulseIn() returns in microseconds
  distance = (duration * 0.034)/ 2; // (duration (microseconds) * speed of sound (10^-4 m/s)) / 2 computes distance in cm
  distance_s = distance;
  Serial.println("Distance: " + String(distance) + " cm");
  if(distance < 25 && !stabilizing) { // If the distance is less than 25 cm, an object is detected
    if(audioPlayer.available()) {
      if(audioPlayer.readType() == DFPlayerPlayFinished) {
        audioPlayer.play(5);
      }
    }
    objectDetected = true;
    digitalWrite(laser, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Object incoming!");
    lcd.setCursor(0, 1);
    lcd.print("Distance: " + String(distance) + " cm  ");
  }
  else {
    objectDetected = false;
    digitalWrite(laser, LOW);
  }
}

void temp_hum() {
  DHT.read11(DHTPIN);
  temp_s = DHT.temperature;
  hum_s = DHT.humidity;
  Serial.println("Temperature: " + String(DHT.temperature) + " Degrees Celsius");
  Serial.println("Humidity: " + String(DHT.humidity) + " %");
  if (!stabilizing && !objectDetected && !micActive) {
    lcd.setCursor(0, 0);
    lcd.print("Temp. in C:   " + String(int(DHT.temperature)));
    lcd.setCursor(0, 1);
    lcd.print("Humidity %:   " + String(int(DHT.humidity)));
  }
}

void water() {
  const int waterThreshold = 100;
  int waterLevel = analogRead(waterSensorPin);
  Serial.println("Water Level: " + String(waterLevel));
  if((waterLevel > waterThreshold) && !raining && !stabilizing && !objectDetected) {
    raining = true;
    Serial.println("It is raining!");
    lcd.setCursor(0, 0);
    lcd.print("It is raining! ");
    delay(2000);
  }
  else if((waterLevel < waterThreshold) && raining && !stabilizing && !objectDetected) {
    raining = false;
    Serial.println("No rain detected!");
    lcd.setCursor(0, 0);
    lcd.print("It is no longer ");
    lcd.setCursor(0, 1);
    lcd.print("raining!        ");
    delay(2000);
  }

  raining_s = raining;
}

void mic() {
  const int micThreshold = 125;
  const int baseline = 337;
  int sample = analogRead(micPin);
  int level = abs(sample - baseline);

  if((level > micThreshold) && !objectDetected) {
    if(audioPlayer.available()) {
      if(audioPlayer.readType() == DFPlayerPlayFinished) {
        audioPlayer.play(6);
      }
    }
    Serial.print("Voice detected! Level: ");
    Serial.println(level);
    lcd.setCursor(0, 0);
    lcd.print("MIC ACTIVE!     ");
    lcd.setCursor(0, 1);
    lcd.print("Transmitting... ");
    delay(2000);
  }
}

void loop() {
  if(DMPReady && mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    aircraftStabilizer();
  }
  ultra_sonic_sensor();
  temp_hum();
  water();
  if(digitalRead(micButton) == LOW) {
    micActive = true;
    Serial.println("Mic Active!");
    mic();
  }
  else {
    micActive = false;
    Serial.println("Mic Inactive!");
  }

  String fullMessage = begin_marker +
  message +       // Field 1: Test Data
  delim1 +
  String(distance_s) + // Field 2: Distance (converted to String)
  delim2 +
  String(raining_s) +
  delim3 +
  String(yaw_s)+
  delim4+
  String(pitch_s)+
  delim5 +
  String(roll_s)+
  delim6+
  String(temp_s)+
  delim7+
  String(hum_s)+
  end_marker;
  Serial1.println(fullMessage);
}