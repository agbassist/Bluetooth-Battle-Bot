#include <Adafruit_BLE_UART.h>
#include "DFRobotDFPlayerMini.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include "SoftwareSerial.h"

// BLUETOOTH

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
#define BUCKETSIZE 64
#define SPEAKERPIN 7

int on = 0;
byte last_ctl;


Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);


// MOTORS

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
// You can also make another motor on port M2
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);

// BEATS

DFRobotDFPlayerMini myDFPlayer;

void setup() {
  SoftwareSerial mySoftwareSerial(5, 6); 
  mySoftwareSerial.begin(9600);
  myDFPlayer.begin(mySoftwareSerial);
  myDFPlayer.volume(20);
  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Bluetooth control test."));

  BTLEserial.setDeviceName("ABCSBOT");
  BTLEserial.begin();
  
  // Set a default frequency
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  left_motor->setSpeed(200);
  right_motor->setSpeed(200);
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  // turn on motor
  left_motor->run(RELEASE);
  right_motor->run(RELEASE);

  // Set up power for the speaker
  pinMode(SPEAKERPIN, OUTPUT);
  digitalWrite(SPEAKERPIN, HIGH);
  
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
        left_motor->setSpeed(0);
        right_motor->setSpeed(0);
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
      byte auth_byte = BTLEserial.read();
      Serial.print("CTL: "); Serial.println(auth_byte);
      
      // Left_input
      byte left_byte = BTLEserial.read();
      Serial.print("LEFT: "); Serial.println(left_byte);
      
      // Right input
      byte right_byte = BTLEserial.read();
      Serial.print("RIGHT: "); Serial.println(right_byte);
      
      // Right input
      byte ctl_byte = BTLEserial.read();
      Serial.print("EXTRA: "); Serial.println(ctl_byte);

      if(ctl_byte^last_ctl){
       
      if (1 == (ctl_byte & 1)){
          if(0 == on){
            myDFPlayer.play(1);
            myDFPlayer.enableLoopAll();
            on = 1;  
          }
          else{
            myDFPlayer.start();
          }
      }
      else if( 0 == (ctl_byte & 1)){
          myDFPlayer.pause();
      }

      last_ctl = ctl_byte;      

      }
      
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
      int speed_left, speed_right;
      
      if(left_byte > 127){
        speed_left = 2 * (left_byte-128);
        left_motor->run(BACKWARD);
      }
      else{
        speed_left = 2 * (127 - left_byte);
        left_motor->run(FORWARD);
      }
      if(right_byte > 127){
        speed_right = 2 * (right_byte - 128);
        right_motor->run(BACKWARD);
      }
      else{
        speed_right = 2 * (127 - right_byte);
        right_motor->run(FORWARD);
      }
      
      left_motor->setSpeed( mapSpeed( speed_left, BUCKETSIZE ) );  
      right_motor->setSpeed(mapSpeed( speed_right, BUCKETSIZE ));

    }
  }
}

int mapSpeed( int input, short bucketsize ) {
  return ( bucketsize * (input/bucketsize)); 
}
