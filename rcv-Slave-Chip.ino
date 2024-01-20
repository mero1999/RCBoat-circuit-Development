#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MechaQMC5883.h>

#define MPU6050_ADDRESS 0x68

MechaQMC5883 qmc;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Variables for calibration
int calibrateIterations = 1000;
float calibrateHeading = 110.62;  // Calibration variable for GY271
float calibrateAccX, calibrateAccY, calibrateAccZ; // Calibration variables MPU6050 Accel
float gyroDriftX = 0.0, gyroDriftY = 0.0; // Calibration variables MPU6050 Gyro
float threshold = 3.0; // // Threshold for MPU6050 Accel

const int SDA_PIN = 2;  // set SDA to D4 (pin 2 nodemcu esp8266)
const int SCL_PIN = 5;  // set SCL to D8 (pin 5 nodemcu esp8266)

Servo Stear;  // D2
Servo Clamp;  // D3
const int StearingPin = 0;  // Stearing Pin D2
const int ClampingPin = 4;  // ClampingPin D3

const int MoD1 = 13;        // Motor Direction D7
const int MoD2 = 12;        // Motor Direction D6
const int SpeedPIN = 14;    // Motor Speed pin D5
String running;             // Motor status

unsigned long RNTime; //start time variable for Motor status change delay (3000 ms)

float roll = 0.0;   // Declare roll at a higher scope
float pitch = 0.0;  // Declare pitch at a higher scope


void setup() {
  Serial.begin(115200);
  initPins();
  initMPU();
  qmc.init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  //calibrateGY271(); //calibration value: 110.62
}

void loop() {
  handleSerial();
  handleMPU();
  handleGY271();
  smartDelay(300);
}
int splitMessage(String message, char delimiter, String parts[]) {
  int partIndex = 0;
  int startIndex = 0;
  int endIndex = message.indexOf(delimiter);

  while (endIndex != -1) {
    parts[partIndex] = message.substring(startIndex, endIndex);
    partIndex++;
    startIndex = endIndex + 1;
    endIndex = message.indexOf(delimiter, startIndex);
  }

  // Capture the last part of the message
  parts[partIndex] = message.substring(startIndex);

  return partIndex + 1;  // Return the number of parts found
}
void handleSerial() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    //Serial.println(msg);
    // Split the message by commas
    String parts[4];  // Gear, speed, Stearing angle, clamp angle
    int n = splitMessage(msg, ',', parts);
    if (n > 2) {
      if (parts[0] == "D") {
        if (running == "R") {
          analogWrite(SpeedPIN, 0);
          delay(25);
        }
        digitalWrite(MoD1, LOW);
        digitalWrite(MoD2, LOW);
        analogWrite(SpeedPIN, parts[1].toInt());
        running = "D";
      } else if (parts[0] == "R") {
        if (running == "D") {
          analogWrite(SpeedPIN, 0);
          delay(25);
        }
        digitalWrite(MoD1, HIGH);
        digitalWrite(MoD2, HIGH);
        analogWrite(SpeedPIN, parts[1].toInt());
        running = "R";
      } else if (parts[0] == "N") {
        if (running = "R") {
          analogWrite(SpeedPIN, 0);
          RNTime = millis();
          running = "N";
        } else {
          analogWrite(SpeedPIN, 0);
          delay(25);
          digitalWrite(MoD1, LOW);
          digitalWrite(MoD2, LOW);
          running = "N";
        }
      }
      Stear.write(parts[2].toInt());
      Clamp.write(parts[3].toInt());
    }
  }
  if (millis() - RNTime > 3000 && running == "N") {
    digitalWrite(MoD1, LOW);
    digitalWrite(MoD2, LOW);
  }
}

void calibrateGY271() {
  Serial.println("Calibrating GY271. Please rotate the sensor 360 degrees in a level plane.");
  for (int i = 0; i < calibrateIterations; i++) {
    int x, y, z, a;
    qmc.read(&x, &y, &z);
    a = qmc.azimuth(&y, &x);
    float headingDegrees = atan2(y, x) * 180 / PI;
    calibrateHeading += headingDegrees;
    smartDelay(20);
  }
  calibrateHeading /= calibrateIterations;
  Serial.println("GY271 Calibration Complete!");
  Serial.println("calibrateHeading: " + (String)calibrateHeading);
}
void handleGY271() {
  int x,y,z,a;
  String cardinal;
  qmc.read(&x,&y,&z);
  float headingDegrees = (atan2(x, y) * 180 / PI ) - calibrateHeading;

  if (headingDegrees < 0) {
    headingDegrees += 360;
  }
  if (headingDegrees > 348.75 || headingDegrees < 11.25) {
    cardinal = " N";
  }
  else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
    cardinal = " NNE";
  }
  else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
    cardinal = " NE";
  }
  else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
    cardinal = " ENE";
  }
  else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
    cardinal = " E";
  }
  else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
    cardinal = " ESE";
  }
  else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
    cardinal = " SE";
  }
  else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
    cardinal = " SSE";
  }
  else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
    cardinal = " S";
  }
  else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
    cardinal = " SSW";
  }
  else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
    cardinal = " SW";
  }
  else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
    cardinal = " WSW";
  }
  else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
    cardinal = " W";
  }
  else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
    cardinal = " WNW";
  }
  else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
    cardinal = " NW";
  }
  else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
    cardinal = " NNW";
  }
  //Serial.println("direction: " + (String)x  +", "+ (String)y +", "+ (String)z +", "+ (String)a);
  //Serial.print("Heading: ");
  Serial.println("\n" + (String)headingDegrees + " " + cardinal);
}
void initMPU() {
  // Initialize MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  accelgyro.initialize();
  //calibrateMPU6050();
}
void handleMPU() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);  // Read 14 registers total

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  // Get calibrated values
  float accX = ax - calibrateAccX;
  float accY = ay - calibrateAccY;
  float accZ = az - calibrateAccZ;

  // Calculate roll and pitch using accelerometer data
  float rollAcc = atan2(accY, accZ) * 180.0 / M_PI;
  float pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / M_PI;

  // Print accelerometer-based tilt angles
  Serial.print(rollAcc);
  Serial.print(", ");
  Serial.print(pitchAcc);
}
void initPins() {
  pinMode(MoD1, OUTPUT);
  digitalWrite(MoD1, LOW);
  pinMode(MoD2, OUTPUT);
  digitalWrite(MoD2, LOW);
  pinMode(SpeedPIN, OUTPUT);
  analogWrite(SpeedPIN, 0);
  Stear.attach(StearingPin, 500, 2500);
  Clamp.attach(ClampingPin, 500, 2500);
  Stear.write(90);
  Clamp.write(90);
  Serial.println("Ready");
}
void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050. Please ensure that the sensor is stationary.");
  for (int i = 0; i < calibrateIterations; i++) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true);  // Read 14 registers total

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    smartDelay(20);
  }
  calibrateAccX /= calibrateIterations;
  calibrateAccY /= calibrateIterations;
  calibrateAccZ /= calibrateIterations;

  Serial.print("\ncalibrateAccX: ");
  Serial.println(calibrateAccX);
  Serial.print("\ncalibrateAccY: ");
  Serial.println(calibrateAccY);
  Serial.print("\ncalibrateAccZ: ");
  Serial.println(calibrateAccZ);
}
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    handleSerial();
  } while (millis() - start < ms);
}