#include <JeeLib.h>
#include <RFM69.h>
#include <SPI.h>

#define DEBUG

// RFM69 settings
#define NODEID      50
#define NETWORKID   5
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY  "1234567890123456"  //Encrypt key must be 16 characters
#define IS_RFM69HW

// Watchdog
ISR(WDT_vect) {Sleepy::watchdogEvent();}

RFM69 radio;

//Data table
typedef struct {
  int    temperature;  //Temperature (°C)
  int    humidity;     //Humidity (%)
  int    voltage;      //Voltage of battery capacity (v)
  byte   measure;      //Measure number (from 0 to 250, +1 each new measurement)
  } Payload;
Payload dataReceived;

const char VERSION[] = "RFM69 receiver";

/**************************************************************************************/
/* Initialization                                                                     */
/**************************************************************************************/
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.print("Starting ");
    Serial.println(VERSION);
  #endif

  // Initialize RFM69 driver
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(true);

  #ifdef DEBUG
    char buff[50];
    sprintf(buff, "Listening at %d Mhz...\n", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(buff);
    radio.readAllRegs();
    Serial.flush();
  #endif

  delay(1000);
}

/**************************************************************************************/
/* Main loop                                                                          */
/**************************************************************************************/
void loop() {
  //Check if data have been received
  if (radio.receiveDone()) {
    int dataValid = 0;
    if (radio.DATALEN != sizeof(dataReceived)) {
      dataValid = 0;
      #ifdef DEBUG
        Serial.print("Invalid payload received, not matching Payload struct!");
      #endif
    } else {
      dataValid = 1;
      dataReceived = *(Payload*)radio.DATA;
    }

    //Send an ACK if required by the node
    if (radio.ACKRequested()) {
      //radio.SENDERID = NODEID;
      radio.sendACK();
      #ifdef DEBUG
        Serial.print("[ACK-sent]");
      #endif
    }
  
    //Display information on serial line if required
    #ifdef DEBUG
      Serial.print("[RSSI:");
      Serial.print(radio.RSSI, DEC);
      Serial.print("]");
      Serial.print("[TEMP:");
      Serial.print(dataReceived.temperature / 10, DEC);
      Serial.print("]");
      Serial.print("[HUMI:");
      Serial.print(dataReceived.humidity / 10, DEC);
      Serial.print("]");
      Serial.print("[VOLTAGE:");
      Serial.print(dataReceived.voltage / 10, DEC);
      Serial.print(" v]");
      Serial.print("[MEASURE:");
      Serial.print(dataReceived.measure);
      Serial.print("] ");
      Serial.println();
    #endif
    }
  }
