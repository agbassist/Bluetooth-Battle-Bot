#include <Adafruit_BLE_UART.h>

/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// BLUETOOTH

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);


// MOTORS

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);



void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Bluetooth control test."));

  BTLEserial.setDeviceName("NEWNAME");
  BTLEserial.begin();
  
  // Set a default frequency
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(200);
  myMotor2->setSpeed(200);
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  
  // Buckets of speed on range [0,255]
  BUCKETSIZE = 32;
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      // Control_input
      char ctl_byte = BTLEserial.read();
      
      // Left_input
      char left_byte = BTLEserial.read();
      
      // Right input
      char right_byte = BTLEserial.read();
      /*
      char c = BTLEserial.read();
      BTLEserial.read();

      Serial.println(c);
      
      char motor_left = c & 0x07;
      
      char motor_right = c & 0x38;
      motor_right = motor_right >> 3;
      
      char control = c & 0xD0;
      control = control >> 6;
      */
      
      // Actual values for motor speeds
      char speed_left, speed_right;
      
      if(left_byte > 0b01111111){
        speed_left = 2 * (left_byte-128);
        myMotor->run(FORWARD);
      }
      else{
        speed_left = 2 * left_byte;
        myMotor->run(BACKWARD);
      }
      if(right_byte > 0b01111111){
        speed_right = 2 * (right_byte - 128);
        myMotor2->run(FORWARD);
      }
      else{
        speed_right = 2 * right_byte;
        myMotor2->run(BACKWARD);
      }
      
      myMotor->setSpeed( mapSpeed( speed_left, BUCKETSIZE ) );  
      myMotor2->setSpeed(mapSpeed( speed_right, BUCKETSIZE ));

      delay(500);

      // Stop until next command sequence
      myMotor->setSpeed(0);
      myMotor2->setSpeed(0);
      /*
      myMotor->setSpeed(0);  
      myMotor2->setSpeed(0);
      myMotor->run(RELEASE);
      myMotor2->run(RELEASE);
      delay(1000);
      */
    }
  }
}

int mapSpeed( char input, short bucketsize ) {
  return ( bucketsize * (input/bucketsize)); 
}

