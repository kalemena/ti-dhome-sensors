#include <JeeLib.h>
#include <RFM69.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DEBUG

// RFM69 settings
#define GATEWAYID   50
#define NODEID      35
#define NETWORKID   5
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY  "1234567890123456"  //Encrypt key must be 16 characters
// #define IS_RFM69HW

#define DHTPIN 3
#define DHTTYPE    DHT22

// Watchdog
ISR(WDT_vect) {Sleepy::watchdogEvent();}

RFM69 radio;
DHT_Unified dht(DHTPIN, DHTTYPE);

//Declare local constants
#define LOOP_DELAY   1000

//Data table
typedef struct {
  int    temperature;  //Temperature (°C)
  int    humidity;     //Humidity (%)
  int    voltage;      //Voltage of battery capacity (v)
  byte   measure;      //Measure number (from 0 to 250, +1 each new measurement)
  } Payload;
Payload dataToSend;

const char VERSION[] = "RFM69 transmitter";

/**************************************************************************************/
/* Initialization                                                                     */
/**************************************************************************************/
void setup() {
  //Open a serial connection to display values
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.print("Starting ");
    Serial.println(VERSION);
  #endif

  //Initialize data
  dataToSend.measure = 0;

  //Initialize RFM69 driver
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.encrypt(ENCRYPTKEY);

  #ifdef DEBUG
    radio.readAllRegs();
    Serial.flush();
  #endif

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));

  delay(1000);
  }

/**************************************************************************************/
/* Main loop                                                                          */
/**************************************************************************************/
void loop() {
  // Gather Payload informations
  dataToSend.voltage = getVoltage() / 10.0;
  //dataToSend.temperature = radio.readTemperature(0);

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    dataToSend.temperature = event.temperature * 10;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    dataToSend.humidity = event.relative_humidity * 10;
  }

  //Insert number of measure
  dataToSend.measure++;
  if (dataToSend.measure > 250) {dataToSend.measure = 0;}

  //Send data on serial link
  #ifdef DEBUG
    Serial.print(dataToSend.temperature, 1);
    Serial.print(" degC");
    Serial.print("   ");
    Serial.print(dataToSend.voltage);
    Serial.print(" v");
    Serial.print("   (");
    Serial.print(dataToSend.measure);
    Serial.println(")");
    Serial.flush();
  #endif

  //Send data
  if (radio.sendWithRetry(GATEWAYID, (const void*)(&dataToSend), sizeof(dataToSend))) {
    #ifdef DEBUG
      Serial.println("Transmission done without problem !");
      Serial.flush();
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("Error with transmission !");
      Serial.flush();
    #endif
  }
  
  //Waiting time before next measurement (minimum 3 seconds)
  radio.sleep();
  //delay(LOOP_DELAY);
  Sleepy::loseSomeTime(LOOP_DELAY);
}

/**************************************************************************************/
/* Allows to get the real Vcc (return value * 100).                                   */
/**************************************************************************************/
int getVoltage() {
  const long InternalReferenceVoltage = 1056L;
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  delay(50);  // Let mux settle a little to get a more stable A/D conversion
  //Start a conversion  
  ADCSRA |= _BV( ADSC );
  //Wait for it to complete
  while (((ADCSRA & (1<<ADSC)) != 0));
  //Scale the value
  int result = (((InternalReferenceVoltage * 1023L) / ADC) + 5L) / 10L;
  return result;
}
