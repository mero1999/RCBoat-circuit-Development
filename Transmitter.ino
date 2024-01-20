#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define D1 5   // CE to pin3 NRF
#define D2 4   // CSN to pin4 NRF
#define D5 14  // SCK to pin5 NRF
#define D6 12  // MISO to pin7 NRF
#define D7 13  // MOSI to pin6 NRF

typedef struct {
  float Latitude = 0.0, Longitude = 0.0, Altitude = 0.0, Speed = 0.0, Direction = 0.0, roll = 0.0, pitch = 0.0;
  char CardinalDirection[4] = "***";
} Data;

typedef struct {
  char text[28] = "";
} sndData;

RF24 radio(D1, D2);  // CE-CSN
const byte rxAddr[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
  Serial.println("\nReady!");
}

void loop() {
  checkSerial();
}

void checkSerial() {
  while (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    if (receivedData.startsWith("req")) {
      send("request");
    } else {
      send(receivedData);
    }
  }
}

void send(String msg) {
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
  sndData data;
  msg.toCharArray(data.text, sizeof(data.text));

  if (radio.write(&data, sizeof(data))) {
    Serial.println("Sent -> " + String(data.text));
  } else {
    Serial.println("Communication Lost!, msg -> " + String(data.text));
  }

  if (String(data.text).startsWith("req")) {
    radio.openReadingPipe(1, rxAddr);
    radio.startListening();
    unsigned long timeout = millis() + 1000;
    while (!radio.available() && millis() < timeout) {
      delay(5);
    } 
    if (radio.available()) {
      Data replyData;
      radio.read(&replyData, sizeof(replyData));
      Serial.println("Reply ->\nLocation: " + (String)replyData.Latitude + ", " + (String)replyData.Longitude + ", " + (String)replyData.Altitude + "m\nDirection: " + (String)replyData.Direction + " : " + (String)replyData.CardinalDirection + "\nSpeed: " + (String)replyData.Speed + "\n Roll: " + (String)replyData.roll + "\t Pitch: " + (String)replyData.pitch);
    } else {
      Serial.println("Error: No Reply!");
    }
  }
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}
