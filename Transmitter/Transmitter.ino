#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin assignments for the NRF24L01 module
#define D1 5   // CE pin to connect to NRF module
#define D2 4   // CSN pin to connect to NRF module
#define D5 14  // SCK pin to connect to NRF module
#define D6 12  // MISO pin to connect to NRF module
#define D7 13  // MOSI pin to connect to NRF module

// Struct to hold the data structure for transmission
typedef struct {
  float Latitude = 0.0, Longitude = 0.0, Altitude = 0.0, Speed = 0.0, Direction = 0.0, roll = 0.0, pitch = 0.0;
  char CardinalDirection[4] = "***";
} Data;

// Struct to hold data to be sent over the radio
typedef struct {
  char text[28] = "";
} sndData;

// NRF24L01 radio instance
RF24 radio(D1, D2);              // Connect NRF CE to D1 and CSN to D2
const byte rxAddr[6] = "00001";  // Receiver address

void setup() {
  Serial.begin(9600);

  // Initialize NRF24L01 radio
  radio.begin();
  radio.setRetries(15, 15);
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();

  Serial.println("\nBoat Controller Ready!");
}

void loop() {
  checkSerial();
}

// Function to check for incoming data from serial
void checkSerial() {
  while (Serial.available() > 0) {
    // Read data from Serial
    String receivedData = Serial.readStringUntil('\n');

    // If a request is received, send a predefined message
    if (receivedData.startsWith("req")) {
      send("request");
    } else {
      send(receivedData);
    }
  }
}

// Function to send data over the radio
void send(String msg) {
  // Set up the NRF24L01 module for transmission
  radio.openWritingPipe(rxAddr);
  radio.stopListening();

  // Prepare data structure for radio transmission
  sndData data;
  msg.toCharArray(data.text, sizeof(data.text));

  // Attempt to send data over radio
  if (radio.write(&data, sizeof(data))) {
    Serial.println("Sent -> " + String(data.text));
  } else {
    Serial.println("Communication Lost!, msg -> " + String(data.text));
  }

  // If the message is a request, listen for a reply
  if (String(data.text).startsWith("req")) {
    radio.openReadingPipe(1, rxAddr);
    radio.startListening();
    unsigned long timeout = millis() + 1000;

    // Wait for a reply
    while (!radio.available() && millis() < timeout) {
      delay(5);
    }

    if (radio.available()) {
      // Receive and decode the reply data
      Data replyData;
      radio.read(&replyData, sizeof(replyData));

      // Print the received data
      Serial.println("Reply ->\nLocation: " + (String)replyData.Latitude + ", " + (String)replyData.Longitude + ", " + (String)replyData.Altitude + "m\nDirection: " + (String)replyData.Direction + " : " + (String)replyData.CardinalDirection + "\nSpeed: " + (String)replyData.Speed + "\n Roll: " + (String)replyData.roll + "\t Pitch: " + (String)replyData.pitch);
    } else {
      Serial.println("Error: No Reply!");
    }
  }

  // Reset radio for future transmissions
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}
