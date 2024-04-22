/*********************************************************************

Embedded System software for the OptiDrop Controller (LysanDas Engineering Group - Atlanta, Georgia)

*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "Adafruit_VL53L0X.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

int sensorValue = 0;  // value read from the pot

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
#define sw1 A0
#define sw2 A1
#define sw3 A2
#define tilt A3
int tiltVal = 0;
bool sw1Bool = 1;
int constState = 0;
bool sw2Bool = 1;
int constState2 = 0;
bool sw3Bool = 1;
int constState3 = 0;

const int analogInPin = A6;  // Analog input pin that the potentiometer is attached to
//const int analogOutPin = 9; 
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  pinMode(sw1, INPUT);

}


/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  sensorValue = analogRead(analogInPin);

  sw1Bool = !digitalRead(sw1);
  sw2Bool = !digitalRead(sw2);
  sw3Bool = !digitalRead(sw3);
  if (sw1Bool == 1 &&  constState == 2) {
        analogWrite(4, 0);
        analogWrite(5, 0);  
        constState = 0;
  }
  if (sw2Bool == 1 &&  constState2 == 2) {
        analogWrite(6, 0);
        analogWrite(7, 0);  
        constState2 = 0;
  }
  if (sw3Bool == 1 &&  constState3 == 2) {
        analogWrite(8, 0);
        analogWrite(9, 0);  
        constState3 = 0;
  }

  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    char num = (char) buttnum;
    if (pressed) {
      Serial.println(" pressed");

      // Close constrictor button (button "1" on number pad)
      if (buttnum == 1) {
        Serial.println("close");
        analogWrite(4, 255);
        analogWrite(5, 0);
        constState = 1;
      } 
      
      // Open constrictor button (button "2" on number pad)
      else if (buttnum == 2 && sw1Bool == 0) {
        Serial.println("open");
        analogWrite(4, 0);
        analogWrite(5, 255);
        constState = 2;
      } 
      
      //////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////
      // ADD CALIBRATION CODE WITHIN THIS CLAUSE BELOW
      // CALIBRATION BUTTON (BUTTON "3" ON NUMBER PAD)
      // add code within this 'else if' clause and test by pressing "3 button" on number pad
      //
      // controls:
      // Pin 3 - Close constrictor motor
      // Pin 4 - Open constrictor motor
      // Pin 5 - Move Right motor
      // Pin 6 - Move Left motor
      // Pin 7 - Move Up motor
      // Pin 8 - Move Down motor 
      //////////////////////////////////////////////////////////////////////////////////////
  
      //////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////
      
      // Left button on trackpad
      else if (buttnum == 7 && sw2Bool == 0) {
        Serial.println("left");
        analogWrite(6, 0);
        analogWrite(7, 255);
        constState2 = 2;
      } 
      
      // Right Button on trackpad
      else if (buttnum == 8 ) {
        Serial.println("right");
        analogWrite(6, 255);
        analogWrite(7, 0);
      } 
      
      // Up Button on trackpad
      else if (buttnum == 5 && sw3Bool == 0) {
        Serial.println("up");
        analogWrite(8, 0);
        analogWrite(9, 255);
        constState3 = 2;
      } 
      
      // Down Button on trackpad
      else if (buttnum == 6) {
        Serial.println("down");
        analogWrite(8, 255);
        analogWrite(9, 0);
      }

      else if (buttnum == 3) {

        ///////////////////////////// reset mode - go to starting position in upper left
        while (sw2Bool == 0) {
            sw2Bool = !digitalRead(sw2);
            Serial.println("left");
            analogWrite(6, 0);
            analogWrite(7, 255);          
        }
        analogWrite(6, 0);
        analogWrite(7, 0);

        while (sw3Bool == 0) {
            sw3Bool = !digitalRead(sw3);
            Serial.println("up");
            analogWrite(8, 0);
            analogWrite(9, 255);
        }
        analogWrite(8, 0);
        analogWrite(9, 0);  

        /////////////////////////////////////// saved head profile mode

        // left eye 
        int set = 0;
        while (set == 0) {
          analogWrite(8, 255);
          analogWrite(9, 0);
          delay(1200);
          set = 1;
        }
        analogWrite(8, 0);
        analogWrite(9, 0);
        set = 0;
        while (set == 0) {
          analogWrite(6, 255);
          analogWrite(7, 0);
          delay(2000);
          set = 1;
        }
        analogWrite(6, 0);
        analogWrite(7, 0);   

        // drop 1
        ///IMPLEMENT
        set = 0;
        while (set == 0) {
          VL53L0X_RangingMeasurementData_t measure;
          tiltVal = analogRead(A3);

          if (tiltVal > 150) {
              lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
              Serial.println(measure.RangeMilliMeter);
              if (measure.RangeMilliMeter > 72) {
                analogWrite(4, 255);
                analogWrite(5, 0);
              }
              analogWrite(4, 0);
              analogWrite(5, 0);
              delay(200);
              analogWrite(4, 0);
              analogWrite(5, 255);
              delay(200);
          } 
        }

      

        // right eye
        // IMPLEMENT

        // drop 2
        // IMPLEMENT 
      }
     


    } 
    
    // When no buttons are being pressed, do not move motors at all
    else {
      Serial.println("No Buttons pressed");
      analogWrite(4, 0);
      analogWrite(5, 0);
      analogWrite(6, 0);
      analogWrite(7, 0);
      analogWrite(8, 0);
      analogWrite(9, 0);
    }

  }

}
