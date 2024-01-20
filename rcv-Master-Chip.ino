//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NodeMcu reciever module code (Boat side)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//GPS
TinyGPSPlus gps;
SoftwareSerial ss(D3, D4);
float Latitude = 0.0, Longitude = 0.0, Altitude = 0.0, Speed = 0.0, Direction = 0.0;
String CardinalDirection = "***";
//MPU6050 data
float roll = 0.0, pitch = 0.0;
//NRF
#define D1 5                     // CE to pin3 NRF
#define D2 4                     // CSN to pin4 NRF
#define D5 14                    // SCK to pin5 NRF
#define D6 12                    // MISO to pin7 NRF
#define D7 13                    // MOSI to pin6 NRF
RF24 radio(D1, D2);              //CE-CSN
const byte rxAddr[6] = "00001";  //RX address
typedef struct {
  float Latitude = 0.0, Longitude = 0.0, Altitude = 0.0, Speed = 0.0, Direction = 0.0, roll = 0.0, pitch = 0.0;
  char CardinalDirection[4] = "***";
} Data;
typedef struct {
  char text[28] = "";
} rcvData;

void setup() {
  Serial.begin(115200);
  initRadio();
  initGPS();
}
void loop() {
  updateRadio();
  updateGPS();
  handleSerial();
}
void handleSerial() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    // Split the message by commas
    String parts[2];  
    splitMessage(msg, ',', parts);
    //Serial.println("Roll: " + parts[0] + "\t pitch: " + parts[1]);
    roll = parts[0].toFloat();
    pitch = parts[1].toFloat();
  }
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
void initRadio() {
  radio.begin();
  radio.setRetries(15, 15);
  radio.openReadingPipe(0, rxAddr);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.setChannel(115);
  radio.startListening();
}
void initGPS() {
  ss.begin(9600);
  //setHome();
}
void updateGPS() {
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      refreshGPSdata();
    }
  }
}
void setHome(){
  while(!gps.location.isValid()){
    updateGPS();
    smartDelay(500);
  }
  float Lat = gps.location.lat();
  float Lon = gps.location.lng();
  //save to hard mem /later
}
void refreshGPSdata() {
  if (gps.location.isValid()) {
    Latitude = gps.location.lat();
    Longitude = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    Altitude = gps.altitude.meters();
  }
  if (gps.speed.isValid()) {
    Speed = gps.speed.kmph();
  }
  if (gps.course.isValid()) {
    Direction = gps.course.deg();
    CardinalDirection = TinyGPSPlus::cardinal(gps.course.deg());
  }
}
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
void updateRadio() {
  if (radio.available()) {
    rcvData data;
    radio.read(&data, sizeof(data));
    Serial.println("recv -> " + String(data.text));
    if (String(data.text).startsWith("req")) {
      sendData();
    } else {
      Serial.println(data.text);  //Gear, speed, Stearing angle, clamp angle
    }
  }
  smartDelay(5);
}
void sendData() {
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
  Data data;
  data.Latitude = Latitude;
  data.Longitude = Longitude;
  data.Altitude = Altitude;
  data.Speed = Speed;
  data.Direction = Direction;
  data.roll = roll;
  data.pitch = pitch;
  CardinalDirection.toCharArray(data.CardinalDirection, sizeof(data.CardinalDirection));
  smartDelay(10);
  Serial.println("Data size: " + (String)sizeof(data));
  radio.write(&data, sizeof(data));
  radio.openReadingPipe(0, rxAddr);
  radio.startListening();
}