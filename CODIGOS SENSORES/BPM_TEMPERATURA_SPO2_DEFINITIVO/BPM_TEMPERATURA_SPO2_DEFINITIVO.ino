/* This code works with MAX30102 + 128x32 OLED i2c + Buzzer and Arduino UNO
 * It's displays the Average BPM on the screen, with an animation and a buzzer sound
 * everytime a heart pulse is detected
 * It's a modified version of the HeartRate library example
 * Refer to www.surtrtech.com for more details or SurtrTech YouTube channel
 */
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include "spo2_algorithm.h"
#include <SparkFunTMP102.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0, 1);
TMP102 sensor0(0x48); 
MAX30105 particleSensor;


uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

void setup() {  
  Serial.begin(115200);
  
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  
  sensor0.begin();  // Join I2C bus
  sensor0.setFault(0);  // Trigger alarm immediately
  sensor0.setAlertPolarity(0); // Active HIGH
  sensor0.setAlertMode(0); // Comparator Mode.
  sensor0.setConversionRate(3);
  sensor0.setExtendedMode(0);

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.begin(Wire, 0x57); //Use default I2C port, 400kHz speed
  particleSensor.setup();
  //particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  //particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

}

void loop() {
 long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
 bufferLength = 100;          

for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();

  if(irValue > 7000){               
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
      
    }      
  }
      
      
      
      
      
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART

                                 //Also detecting a heartbeat



  String temperature;
  sensor0.wakeup();
  temperature = sensor0.readTempC();
  sensor0.sleep();
  Serial.print("T=");
  Serial.write(temperature[0]);
  Serial.write(temperature[1]);
  //Serial.write(temperature[2]);
  //Serial.write(temperature[3]);
  //Serial.write(temperature[4]); 
  Serial.print(", B=");
  Serial.print(beatsPerMinute);
  Serial.print(", BPM=");
  Serial.print(beatAvg);
  Serial.print(F(", SPO2="));
  Serial.println(spo2-2, DEC); 
    }
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  }



   
}
