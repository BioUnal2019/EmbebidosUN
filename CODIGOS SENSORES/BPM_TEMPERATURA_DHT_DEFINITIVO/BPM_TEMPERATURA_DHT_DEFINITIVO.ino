/* This code works with MAX30102 + 128x32 OLED i2c + Buzzer and Arduino UNO
 * It's displays the Average BPM on the screen, with an animation and a buzzer sound
 * everytime a heart pulse is detected
 * It's a modified version of the HeartRate library example
 * Refer to www.surtrtech.com for more details or SurtrTech YouTube channel
 */
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include <SparkFunTMP102.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

SoftwareSerial mySerial(0, 1);
TMP102 sensor0(0x48); 
MAX30105 particleSensor;

#define DHTPIN 2 
#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
String BBB=String();
String PPP=String();
String SSS=String();
int EEE1;
int EEE2;

int incomingByte = 0; // for incoming serial data

void setup() {  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  //mySerial.begin(115200);// Start serial communication at 9600 baud
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  
  sensor0.begin();  // Join I2C bus
  sensor0.setFault(0);  // Trigger alarm immediately
  sensor0.setAlertPolarity(0); // Active HIGH
  sensor0.setAlertMode(0); // Comparator Mode.
  sensor0.setConversionRate(3);
  sensor0.setExtendedMode(0);

  particleSensor.begin(Wire, 0x57); //Use default I2C port, 400kHz speed
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop() {
 long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
                                           //Also detecting a heartbeat
if(irValue > 7000){                                           //If a finger is detected
    
  if (checkForBeat(irValue) == true)                        //If a heart beat is detected
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;                   //Measure duration between two beats
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);           //Calculating the BPM

    if (beatsPerMinute < 255 && beatsPerMinute > 20)               //To calculate the average we strore some values (4) then do some math to calculate the average
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      BBB=(beatAvg);
      /*
      EEE1=BBB[0];
      EEE2=BBB[1];
      PPP=String(EEE1,BIN)
      SSS=String(EEE2,BIN)*/
    }
  }

}
  if (irValue < 7000){       //If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
     beatAvg='0';
     BBB=(beatAvg);
     }

  sensors_event_t event;
  dht.temperature().getEvent(&event);
    String tem=String();
  String hum=String();
  int tem2;
  int hum2;
   if (isnan(event.temperature)) {
    tem=('0');
  }
  else {
    tem= event.temperature;
    tem2= event.temperature;
    SSS=String(tem2,BIN);
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    hum=('0');
  }
  else {
    hum=event.relative_humidity;
    hum2= event.temperature;
    PPP=String(hum2,BIN);
  }
        
  String temperature;
  sensor0.wakeup();
  temperature = sensor0.readTempC();
  sensor0.sleep();

  
  
  //Serial.print("T=");
  Serial.write(temperature[0]);
  Serial.write(temperature[1]);
  //Serial.write(temperature[2]);
  //Serial.write(temperature[3]);
  //Serial.write(temperature[4]); 
  //Serial.print(", BPM=");
  //Serial.print(beatAvg);
  //Serial.print('-');
  Serial.write(BBB[0]);
  Serial.write(BBB[1]);
  //Serial.print(", Ta=");
  Serial.write(tem[0]);
  Serial.write(tem[1]);
  //USerial.print('-');
  /*Serial1.write(SSS[0]);
  Serial1.write(SSS[1]);
  Serial1.write(SSS[2]);
  Serial1.write(SSS[3]);
  Serial1.write(SSS[4]);
  Serial1.write(SSS[5]);
  Serial1.write(SSS[6]);
  Serial1.write(SSS[7]);
  Serial.print(", H=");*/
  Serial.write(hum[0]);
  Serial.write(hum[1]);
  //Serial.print('-');
  //Serial1.write(PPP[0]);
  //Serial1.write(PPP[1]);
  //Serial1.write(PPP[2]);
  //Serial1.write(PPP[3]);
  //Serial1.write(PPP[4]);
  //Serial1.write(PPP[5]);
  //Serial1.write(PPP[6]);
  //Serial1.write(PPP[7]);
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.println(incomingByte, BIN);
  }
  Serial.println();
}
