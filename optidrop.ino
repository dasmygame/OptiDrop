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


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


// LiDAR drop sensor
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

// motor controls
#define close 4
#define open 5
#define right 6
#define left 7
#define down 8
#define up 9

// switch states
int sw1Bool = 0;
int constState1 = 0;

int sw2Bool = 0;
int constState2 = 0;

int sw3Bool = 0;
int constState3 = 0;

int tiltSensor = A5; 
int tiltSensorValue = 0;

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
  pinMode(13, OUTPUT);

}


/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // constrictor max open switch read
  sw1Bool = digitalRead(sw1);

  // tilt sensor read
  tiltSensorValue = analogRead(tiltSensor);

  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading drop distance measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  Serial.print("Drop Distance (mm): "); Serial.println(measure.RangeMilliMeter);

  if (measure.RangeMilliMeter < 72) {
    Serial.println("open");
    analogWrite(close, 0);
    analogWrite(open, 255);
    delay(200);
  }

  // check if open is at max
  if (sw1Bool == 1 &&  constState1 == 2) {
    analogWrite(4, 0);
    analogWrite(5, 0);  
    constState1 = 0;
  }

  // check if up is at right
  if (sw2Bool == 1 &&  constState2 == 2) {
    analogWrite(6, 0);
    analogWrite(7, 0);  
    constState2 = 0;
  }

  // check if up is at max
  if (sw3Bool == 1 &&  constState3 == 2) {
    analogWrite(7, 0);
    analogWrite(8, 0);  
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
        analogWrite(close, 255);
        analogWrite(open, 0);
        constState1 = 1;
      } 
      
      // Open constrictor button (button "2" on number pad)
      else if (buttnum == 2 && sw1Bool == 0) {
        Serial.println("open");
        analogWrite(close, 0);
        analogWrite(open, 255);
        constState1 = 2;
      } 
      
      //////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////
      // ADD CALIBRATION CODE WITHIN THIS CLAUSE BELOW
      // CALIBRATION BUTTON (BUTTON "3" ON NUMBER PAD)
      // add code within this 'else if' clause and test by pressing "3 button" on number pad
      //
      // controls:
      // Pin 4 - Close constrictor motor
      // Pin 5 - Open constrictor motor
      // Pin 6 - Move Right motor
      // Pin 7 - Move Left motor
      // Pin 8 - Move Up motor
      // Pin 9 - Move Down motor 
      //////////////////////////////////////////////////////////////////////////////////////
      
      else if (buttnum == 3) {
        // Add code for first calibration here
        Serial.println("Calibration A");

        ///////////////////  RESET TO START POSITION  //////////////////////////////////////
        // reset right
        while (sw2Bool == 0) {
          analogWrite(right, 255);
          analogWrite(left, 0);
        }
        analogWrite(right, 0);
        analogWrite(left, 0);

        // reset up
        while (sw2Bool == 0) {
          analogWrite(up, 255);
          analogWrite(down, 0);
        }
        analogWrite(up, 0);
        analogWrite(down, 0);

        //////////////////   END RESET   ///////////////////////////////////////////////////
        
        ////////////////////////////////////////////////////////////////////////////////////
        
        int eyeCount = 0;

        //////////////////   CALIBRATE MODE    /////////////////////////////////////////////

        // calibrate to maximum of 2 eyes
        while (eyeCount < 2) {

          ///////////////////////
          ///// RIGHT EYE ///////
          ///////////////////////
          if (eyeCount = 0) {
            analogWrite(left, 255);
            analogWrite(right, 0);
            delay(3000);
            analogWrite(down, 255);
            analogWrite(up, 0);
            delay(2000);
          }
          analogWrite(left, 0);
          analogWrite(right, 0);
          analogWrite(up, 0);
          analogWrite(down, 0);

          ///////////////////////
          ///// LEFT EYE ///////
          ///////////////////////
          if (eyeCount = 1) {
            analogWrite(left, 255);
            analogWrite(right, 0);
            delay(3000);
          }
          analogWrite(left, 0);
          analogWrite(right, 0);
          analogWrite(up, 0);
          analogWrite(down, 0);


        //////////////////   END CALIBRATE MODE    /////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////

        //////////////////   DROP MODE   ///////////////////////////////////////////////////
        
        // drop mode through tilt sensing
          int dropDone = 0;
          if (tiltSensorValue > 150 && dropDone == 0) {
            int set = 0;
            while (set == 0) {
              Serial.println("close");
              analogWrite(close, 255);
              analogWrite(open, 0);
              
              // check drop distance
              lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
              Serial.print("Drop Distance (mm): "); Serial.println(measure.RangeMilliMeter);

              // if drop is sensed, open constrictor
              if (measure.RangeMilliMeter < 72) {
                Serial.println("open");
                analogWrite(close, 0);
                analogWrite(open, 255);
                delay(1000);
                set = 1;
                dropDone = 1;
              }

            }
          }
          eyeCount += 1; 
        //////////////////   END DROP MODE   ///////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////
        }
      }

      //////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////
      

      // Left button on trackpad
      else if (buttnum == 7) {
        Serial.println("left");
        analogWrite(right, 0);
        analogWrite(left, 255);
      } 
      
      // Right Button on trackpad
      else if (buttnum == 8) {
        Serial.println("right");
        analogWrite(right, 255);
        analogWrite(left, 0);
      } 
      
      // Up Button on trackpad
      else if (buttnum == 5) {
        Serial.println("up");
        analogWrite(down, 0);
        analogWrite(up, 255);
      } 
      
      // Down Button on trackpad
      else if (buttnum == 6) {
        Serial.println("down");
        analogWrite(down, 255);
        analogWrite(up, 0);
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
