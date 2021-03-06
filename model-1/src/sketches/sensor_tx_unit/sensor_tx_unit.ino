#define RF69_COMPAT 0 // define this to use the RF69 driver i.s.o. RF12

#include "DHT.h"
#include <JeeLib.h>
#include <PortsSHT11.h>
#include <PortsBMP085.h>
#include <PortsLCD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <Vcc.h>

#define NODEID    10        // node ID used for this unit
#define NODEGROUP 5         // node GROUP used for this unit
#define REPORT_PERIOD  300  // how often to measure, in tenths of seconds

#define SHT11_PORT     0    // define SHT11 port
#define DHT22_PORT     0    // define DHT22 port (8 ?)
#define DS18B20_1_PORT 4    // WARNING 4=DIO port 1, 5=PORT2, 6=PORT3, 7=PORT4 ... 1-wire temperature sensors
#define DS18B20_2_PORT 0    // WARNING ..
#define LDR_PORT       0    // define LDR on AO,A1,... pin  // WARNING 1=A0, 2=A1, 3=A2, 4=A3
#define LDR_ENABLED    false
#define BMP85_PORT     0    // define BMP85 port
#define SOIL_PORT      0    // A0, A1 ...
#define SOIL_ENABLED   false
#define TOGGLE_1_PORT  0    // digital port
#define TOGGLE_2_PORT  0    // digital port
// #define BATTERY_CORRECTION 3.86/3.71 // Measured Vcc by multimeter divided by reported Vcc
#define BATTERY_CORRECTION 1 // Measured Vcc by multimeter divided by reported Vcc

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

// Type of data to be reported
enum { TEMPERATURE, HUMIDITY, PRESSURE, LIGHT, BATTERY, SOIL, TOGGLE, TYPE_END };

// The scheduler makes it easy to perform various tasks at various times:
enum { REPORT, TASK_END };
static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

struct {
    byte id;  
    byte port; 
    byte type;
    int  value;
} payload;

// count up reports order until next reset
static byte reportCount;

Vcc vcc(BATTERY_CORRECTION);

// Conditional code, depending on which sensors are connected and how:
#if SHT11_PORT
    SHT11 sht11 (SHT11_PORT);
#endif
#if DHT22_PORT
    DHT dht;
#endif
#if DS18B20_1_PORT
    OneWire ds18b20_1 (DS18B20_1_PORT);
    DallasTemperature sensors_1(&ds18b20_1);
    DeviceAddress insideThermometer_1;
    uint8_t oneID_1[8]; 
#endif
#if DS18B20_2_PORT
    OneWire ds18b20_2 (DS18B20_2_PORT);
    DallasTemperature sensors_2(&ds18b20_2);
    DeviceAddress insideThermometer_2;
    uint8_t oneID_2[8]; 
#endif
#if BMP85_PORT
    PortI2C i2c_pressure(BMP85_PORT);
    BMP085 psensor (i2c_pressure, 3); // ultra high resolution
#endif

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// readout all the sensors and other values, and report
static void measureAndReport() {
    payload.id = NODEID;
    
    payload.type = BATTERY;
    payload.value = vcc.Read_Volts() * 1000; 
    payload.port = 1;
    report();

    #if SHT11_PORT
        sht11.measure(SHT11::HUMI, Sleepy::loseSomeTime(32));        
        sht11.measure(SHT11::TEMP, Sleepy::loseSomeTime(32));
        
        float h, t;
        sht11.calculate(h, t);
        payload.type = TEMPERATURE;
        payload.value = 10 * t;
        payload.port = 2;   
        report();
        
        payload.type = HUMIDITY;
        payload.value = 10 * h;
        payload.port = 3;
        report();
    #endif
    #if DHT22_PORT
      float h = dht.getHumidity();
      float t = dht.getTemperature();
      
      String eq= "OK";
      String res = dht.getStatusString();
      if(eq.equals(res)) {
        payload.type = TEMPERATURE;
        payload.value = 10 * t;
        payload.port = 2;   
        report();
        payload.type = HUMIDITY;
        payload.value = 10 * h;
        payload.port = 3;
        report();
      }
    #endif
    #if DS18B20_1_PORT
        payload.type = TEMPERATURE;
        sensors_1.requestTemperatures(); 
        float tempC1 = sensors_1.getTempC(insideThermometer_1);
        payload.value = tempC1 * 10;
        payload.port = 4;
        report();
    #endif
    #if DS18B20_2_PORT
        payload.type = TEMPERATURE;
        sensors_2.requestTemperatures(); 
        float tempC2 = sensors_2.getTempC(insideThermometer_2);
        payload.value = tempC2 * 10;
        payload.port = 5;
        report();
    #endif
    #if BMP85_PORT
        psensor.startMeas(BMP085::TEMP);
        Sleepy::loseSomeTime(16);
        int32_t traw = psensor.getResult(BMP085::TEMP);
    
        psensor.startMeas(BMP085::PRES);
        Sleepy::loseSomeTime(32);
        int32_t praw = psensor.getResult(BMP085::PRES);
        int16_t tempBMP85;
        int32_t pressure;
        psensor.calculate(tempBMP85, pressure);

        payload.type = TEMPERATURE;
        payload.value = tempBMP85;
        payload.port = 6;
        report();
        
        payload.type = PRESSURE;
        payload.value = pressure / 10;
        payload.port = 7;
        report();        
    #endif
    #if LDR_ENABLED
        int32_t LDRReading = 255 - analogRead(LDR_PORT) / 4;

        payload.type = LIGHT;
        payload.value = LDRReading;
        payload.port = 8;
        report();
    #endif
    #if SOIL_ENABLED
      int32_t sensorValue = analogRead(SOIL_PORT);
      //sensorValue = constrain(sensorValue, 600, 1022);
      //int soil = map(sensorValue, 600, 1022, 100, 0);
  
      payload.type = SOIL;
      payload.value = sensorValue;
      payload.port = 9;
      report();
    #endif
    #if TOGGLE_1_PORT
      int toggleState1 = digitalRead(TOGGLE_1_PORT);
      payload.type = TOGGLE;
      payload.value = toggleState1;
      payload.port = 10;
      report();
    #endif
    #if TOGGLE_2_PORT
      int toggleState2 = digitalRead(TOGGLE_2_PORT);
      payload.type = TOGGLE;
      payload.value = toggleState2;
      payload.port = 11;
      report();
    #endif
}

static void report() {
    rf12_sleep(-1);
    while (!rf12_canSend())
        rf12_recvDone();
    rf12_sendNow(0, &payload, sizeof payload);
    rf12_sendWait(RADIO_SYNC_MODE);
    rf12_sleep(0);

    Sleepy::loseSomeTime(100);
}

void setup () {
    rf12_initialize(NODEID, RF12_868MHZ, NODEGROUP);
    rf12_sleep(0); // power down
 
    #if DHT22_PORT
        dht.setup(DHT22_PORT);
    #endif
    #if DS18B20_1_PORT
        sensors_1.begin();
        boolean bAddress1 = sensors_1.getAddress(insideThermometer_1, 0);
        sensors_1.setResolution(insideThermometer_1, 12);
    #endif
    #if DS18B20_2_PORT
        sensors_2.begin();
        boolean bAddress2 = sensors_2.getAddress(insideThermometer_2, 0);
        sensors_2.setResolution(insideThermometer_2, 12);
    #endif
    #if BMP85_PORT
        psensor.getCalibData();
    #endif
    #if LDR_ENABLED
      pinMode(LDR_PORT, INPUT_PULLUP);
    #endif
    #if TOGGLE_1_PORT
      pinMode(TOGGLE_1_PORT, INPUT); 
    #endif
    #if TOGGLE_2_PORT
      pinMode(TOGGLE_2_PORT, INPUT); 
    #endif
   
    reportCount = 0;
    scheduler.timer(REPORT, 0);    // start the measurement loop going
}

void loop () {
  switch (scheduler.pollWaiting()) 
  {
    case REPORT:
	reportCount++;
    
	// reschedule these measurements periodically
	scheduler.timer(REPORT, REPORT_PERIOD);
	measureAndReport();
		    
	//scheduler.timer(REPORT, 0);
	break;            
  }
}
