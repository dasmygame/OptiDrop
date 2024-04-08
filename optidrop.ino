#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <SPI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>
#include "BluefruitConfig.h"
#include <Keypad.h>

const int left = 2;  // the number of the pushbutton pin
const int right = 3;
const int up = 4;
const int down = 5;
const int drop = 6;

const int ledPin = 13;    // the number of the LED pin for testing

int leftState = 0;
int rightState = 0;
int upState = 0;
int downState = 0;
int dropState = 0;

void setup() {
  //analog motor pins
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  
  //buttons to control track/axis movement
  pinMode(left, INPUT);
  pinMode(right, INPUT);
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(drop, INPUT);

  Serial.begin(9600);

}

void loop() {
  delay(100);

  leftState = digitalRead(left);
  rightState = digitalRead(right);
  
  upState = digitalRead(up);
  downState = digitalRead(down);
  dropState = digitalRead(drop);

  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if (leftState == LOW) {
    Serial.println("left");
    analogWrite(A3, 1023);
    analogWrite(A4, 0);
  }

  if (rightState == LOW) {
    Serial.println("left");
    analogWrite(A3, 0);
    analogWrite(A4, 1023);
  } 
  
  if (leftState == HIGH && rightState == HIGH) {
    analogWrite(A3, 0);
    analogWrite(A4, 0);
  }

  if (upState == LOW) {
    // turn LED on:
    Serial.println("left");
    analogWrite(A1, 1023);
    analogWrite(A2, 0);
  

  } 

  if (downState == LOW) {
    // turn LED on:
    Serial.println("left");
    analogWrite(A1, 0);
    
    analogWrite(A2, 1023);
    
  } 

  if (downState == HIGH && upState == HIGH)
  {
    analogWrite(A1, 0);
    analogWrite(A2, 0);
  }

  //Drop eyedrop by closing constrictor
  if (dropState == LOW) {
    Serial.println("left");
    analogWrite(A5, 1023);
    analogWrite(A6, 0);
    
  } else {
    analogWrite(A5, 0);
    analogWrite(A6, 0);
  }
}
