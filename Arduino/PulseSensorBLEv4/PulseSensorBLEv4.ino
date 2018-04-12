/*
  HeartRateBLE.ino
  Written by Federico Sanna
  This example demonstrates the use of the Serial Plotter (Tools->Serial Plotter) to display the heart beats and
  how to send to a smarthphone via BLE information on the heart rate according to bluetooth standards.
  To diplay the heart rate on the serial plotter the board has to be connected via bluetooth first.
  You can adjust the threshold value depending on who you are recording the value on to better capture the beats,
  and you can use the Nordic Toolbox App to receive the heart rate on your phone.
  The heart rate is calculated on values recorded every 10 milliseconds over an interval of 3 seconds, meaning that 
  the heart rate will be updated every 3 seconds.
  In this example BLE_LED shows the status of the board. It will blink every 200 ms when the board is scanning.
  It will be on when the board is connected to a peripheral. It will be off when the board is disconnected.
  This example code is in the public domain.
*/

#include <BLEPeripheral.h>
#include "DFRobot_Heartrate.h"
#include <Wire.h>
#include <MPU6050.h>

#define heartratePin A5

BLEPeripheral blePeripheral;          // BLE Peripheral Device
BLEService heartRateService("180D");  // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic heartRateChar("2A37",  // standard 16-bit characteristic UUID
                                BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
// the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
// https:/developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

DFRobot_Heartrate heartrate(DIGITAL_MODE); ///< ANALOG_MODE or DIGITAL_MODE
MPU6050 mpu;

boolean ledState = false;
boolean freefallDetected = false;
int freefallBlinkCount = 0;

unsigned long bAvg[10];
uint8_t bCount = 0;

void setup() {
  Serial.begin(115200);

  blePeripheral.setLocalName("HeartRateSketch");                     // Set a local name for the BLE device
  blePeripheral.setAdvertisedServiceUuid(heartRateService.uuid());   // add the service UUID
  blePeripheral.addAttribute(heartRateService);                      // Add the BLE Heart Rate service
  blePeripheral.addAttribute(heartRateChar);                         // add the Heart Rate Measurement characteristic
  
  blePeripheral.begin();  //activate BLE device to continuosly transmit BLE advertising packets.
  //Your board will be visible to central devices until it receives a new connection
  Serial.println("Bluetooth device active, waiting for connections...");

  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  
  mpu.setIntFreeFallEnabled(true);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);

  mpu.setFreeFallDetectionThreshold(17);
  mpu.setFreeFallDetectionDuration(2);  
  attachInterrupt(2, doInt, RISING);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
}

void doInt()
{
  freefallBlinkCount = 0;
  freefallDetected = true;  
}

void loop() {
  BLECentral central = blePeripheral.central();   // listen for BLE peripherals to connect

  if (central) {                             //when connected to a central
    Serial.print("Connected to central: ");
    Serial.println(central.address());       // print the central's MAC address:
    //turn on BLE led

    // as long as the central is still connected:
    while (central.connected()) {
//      digitalWrite(13, HIGH);
//      rfid_check();
      heartrate.getValue(heartratePin);
      uint8_t bps = heartrate.getRate();
  
      if(bps)  {
        bAvg[bCount] = bps;
    //    Serial.println(rateValue);
        if(bCount<9){
          bCount++;
        }
        else{
          uint8_t avg = average(bAvg,10);
  
          uint8_t hrmdata[2] = { 0b00000110, avg };
  
          if ( heartRateChar.setValue(hrmdata, sizeof(hrmdata)) ){
            // Note: We use .notify instead of .write!
            // If it is connected but CCCD is not enabled
            // The characteristic's value is still updated although notification is not sent
  
            Serial.print("Heart Rate Measurement updated to: "); Serial.println(avg); 
          }else{
            Serial.println("ERROR: Notify not set in the CCCD or not connected!");
          }
          bCount = 0;
        }
      }
      Vector rawAccel = mpu.readRawAccel();
      Activites act = mpu.readActivites();
    
      Serial.print(act.isFreeFall);
      Serial.print("\n");
      
      if (freefallDetected)
      {
        ledState = !ledState;
    
//        digitalWrite(13, ledState);
    
        freefallBlinkCount++;
    
        if (freefallBlinkCount == 20)
        {
          freefallDetected = false;
          ledState = false;
//          digitalWrite(13, ledState);
        }
      }
      digitalWrite(13, ledState);
      delay(20);
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  delay(20);
}

uint8_t average (unsigned long * array, uint8_t len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  (uint8_t)(sum / len) ;  // average will be fractional, so float may be appropriate.
}

void rfid_check(){
  byte i = 0;
  byte val = 0;
  byte code[6];
  byte checksum = 0;
  byte bytesread = 0;
  byte tempbyte = 0;

  if(Serial.available() > 0) {
    if((val = Serial.read()) == 2) {                  // check for header 
      bytesread = 0; 
      while (bytesread < 12) {                        // read 10 digit code + 2 digit checksum
        if( Serial.available() > 0) { 
          val = Serial.read();
          if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) { // if header or stop bytes before the 10 digit reading 
            break;                                    // stop reading
          }

          // Do Ascii/Hex conversion:
          if ((val >= '0') && (val <= '9')) {
            val = val - '0';
          } else if ((val >= 'A') && (val <= 'F')) {
            val = 10 + val - 'A';
          }

          // Every two hex-digits, add byte to code:
          if (bytesread & 1 == 1) {
            // make some space for this hex-digit by
            // shifting the previous hex-digit with 4 bits to the left:
            code[bytesread >> 1] = (val | (tempbyte << 4));

            if (bytesread >> 1 != 5) {                // If we're at the checksum byte,
              checksum ^= code[bytesread >> 1];       // Calculate the checksum... (XOR)
            };
          } else {
            tempbyte = val;                           // Store the first hex digit first...
          };

          bytesread++;                                // ready to read next digit
        } 
      } 

      // Output to Serial:

      if (bytesread == 12) {                          // if 12 digit read is complete
        Serial.print("5-byte code: ");
        for (i=0; i<5; i++) {
          if (code[i] < 16) Serial.print("0");
          Serial.print(code[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        Serial.print("Checksum: ");
        Serial.print(code[5], HEX);
        Serial.println(code[5] == checksum ? " -- passed." : " -- error.");
        Serial.println();
      }

      bytesread = 0;
    }
  }
}
