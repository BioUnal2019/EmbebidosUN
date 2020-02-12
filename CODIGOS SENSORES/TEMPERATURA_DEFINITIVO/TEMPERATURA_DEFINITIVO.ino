#include <Wire.h> // Used to establied serial communication on the I2C bus
#include <SparkFunTMP102.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(0, 1);
TMP102 sensor0(0x48); 

void setup() {
  Serial.begin(115200); 
  mySerial.begin(115200);// Start serial communication at 9600 baud
  sensor0.begin();  // Join I2C bus

  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  sensor0.setFault(0);  // Trigger alarm immediately

  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  sensor0.setAlertPolarity(0); // Active HIGH

  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  sensor0.setAlertMode(0); // Comparator Mode.

  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor0.setConversionRate(3);

  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor0.setExtendedMode(0);

}

void loop()
{
  String temperature;
  boolean alertPinState, alertRegisterState;
 
  sensor0.wakeup();
  temperature = sensor0.readTempC();
  sensor0.sleep();
  Serial.write(temperature[0]);
  Serial.write(temperature[1]);
  Serial.write(temperature[2]);
  Serial.write(temperature[3]);
  Serial.write(temperature[4]);
  //Serial.println();
  delay(1000);
}
