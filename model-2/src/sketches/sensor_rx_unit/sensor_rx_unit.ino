//#define RF69_COMPAT 1 // define this to use the RF69 driver i.s.o. RF12
#include <Ports.h>
#include <RF12.h>

struct Payload {
    byte id;  
    byte port; 
    byte type;
    int  value   :11;
};

void setup () {
    Serial.begin(57600);
    rf12_initialize(30, RF12_868MHZ, 5); // 868 Mhz, net group 5, node 30
    Serial.println("BOOT OK");
}

void loop () {
    //Serial.println(".");
    delay(100);
    if (rf12_recvDone() && rf12_crc == 0 && rf12_len == sizeof (Payload)) {
        Payload* data = (Payload*) rf12_data;
        
        Serial.print("ROOM ");
        Serial.print((int)data->id);
        Serial.print(' ');
        Serial.print((int)data->port);
        Serial.print(' ');
        Serial.print((int)data->type);
        Serial.print(' ');
        Serial.println((int)data->value);
    }
}
