#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>
#include <SD.h>
#include <EEPROM.h>
#include <FlexCAN_T4.h>
void setup() {
    Serial.begin(9600);
}

void loop(){
    Serial.println("hi");
    delay(50);
}