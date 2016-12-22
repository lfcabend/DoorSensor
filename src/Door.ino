#include <Arduino.h>

#define MY_BAUD_RATE 9600

//#define MY_DEBUG
#define MY_RADIO_NRF24
#define MY_NODE_ID 12

#define DOOR_PIN 3

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_DOOR 2


#include <SPI.h>
#include <MySensors.h>
#include <Vcc.h>

#include <Wire.h>
//#include "Adafruit_HTU21DF.h"


// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin (A5 on UNO)
// Connect SDA to I2C data pin (A4 on UNO)

//Adafruit_HTU21DF htu = Adafruit_HTU21DF();

unsigned long SLEEP_TIME = 60L * 1000L * 60L; // Sleep time between reads (in milliseconds)
unsigned long WAIT_TIME = 5000L;

volatile boolean trigger = false;
const float VccMin        = 2.0*0.6;  // Minimum expected Vcc level, in Volts. Example for 2xAA Alkaline.
const float VccMax        = 2.0*1.5;  // Maximum expected Vcc level, in Volts. Example for 2xAA Alkaline.
const float VccCorrection = 1.0/1.0;  // Measured Vcc by multimeter divided by reported Vcc
Vcc vcc(VccCorrection);

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgDoorTripped(CHILD_ID_DOOR, V_TRIPPED);

float previousTemp = -1;
float previousHum = -1;
float previousbatteryPcnt = -1;

void setup() {
  // put your setup code here, to run once:
//  if (!htu.begin()) {
//    Serial.println("Couldn't find sensor!");
//    while (1);
//  }

  pinMode(DOOR_PIN, INPUT_PULLUP);

}


void presentation()
{

  // Send the Sketch Version Information to the Gateway
  sendSketchInfo("Door Sketch", "1.0");

   // Register all sensors to gw (they will be created as child devices)
//  present(CHILD_ID_HUM, S_HUM);
//  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_DOOR, S_DOOR);
}

void loop() {

  // put your main code here, to run repeatedly:
  int batteryPcnt = (int) vcc.Read_Perc(VccMin, VccMax);

//  float temp = htu.readTemperature();
//  float hum = htu.readHumidity();
  boolean doorOpen = digitalRead(DOOR_PIN) == HIGH;

  #ifdef MY_DEBUG

  Serial.println("loop");
  Serial.print("batterPcnt: " ); Serial.println(batteryPcnt);
//  Serial.print("Temp: "); Serial.print(temp);
//  Serial.print("\t\tHum: "); Serial.println(hum);


  if(digitalRead(DOOR_PIN) == LOW){
    Serial.println("Door closed");
  } else{
    Serial.println("Door open");
  }

  #endif

  if (batteryPcnt != previousbatteryPcnt) {
    sendBatteryLevel(batteryPcnt);
    previousbatteryPcnt = batteryPcnt;
  }

  send(msgDoorTripped.set(doorOpen?"1":"0"));

//  if (hum != previousHum) {
//    previousHum = hum;
//    send(msgHum.set(hum, 1));
//  }
//
//  if (temp != previousTemp) {
//    previousTemp = temp;
//    send(msgTemp.set(temp, 1));
//  }
  if(isTransportOK()){
    sleep(digitalPinToInterrupt(DOOR_PIN), CHANGE, SLEEP_TIME);
  }
  else {
    wait(WAIT_TIME); // transport is not operational, allow the transport layer to fix this
  }

}
