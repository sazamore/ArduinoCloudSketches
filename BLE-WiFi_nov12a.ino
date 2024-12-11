#include "arduino_secrets.h"
/*
  BLE-WIFI (Cloud)

  This code blinks a BLE peripheral about every 5 seconds and updates a corresponding LED Cloud Variable.

  The wifiTask updates the Cloud Variable led. 
  The BLEtaask sends byte data to the LED characteristic, which turns on the LED peripheral (run ArduinoBLE example code LED.ino).

  The circuit:
  - Arduino Nano 33 IoT (controller/central) 
  - Arduino BLE Sense (peripheral)

  This example code is in the public domain.
*/

#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_ConnectionHandler.h>
#include <ArduinoBLE.h>

#include <DHT11.h>
#include "thingProperties.h"
#include "arduino_secrets.h"

//----------------------------------------------------------------------------------------------------------------------
// Project
//----------------------------------------------------------------------------------------------------------------------

#define PROJECT_NAME                       "BLE-WiFi Switch Example"

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs & LocalName
//----------------------------------------------------------------------------------------------------------------------

// https://www.bluetooth.com/specifications/gatt/services/
// https://www.bluetooth.com/specifications/gatt/characteristics/

#define BLE_UUID_BATTERY_SERVICE                  "180F"
#define BLE_UUID_BATTERY_MONITOR                  "2A19"

//========== Add your custom services, characteristic and local name =========================

#define BLE_UUID_SERVICE_LED                      "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_CHAR_SWITCH                      "19B10001-E8F2-537E-4F6C-D104768A1214"
#define LOCALNAME                                 "LED"

// ============================================================================================

//----------------------------------------------------------------------------------------------------------
// App
//----------------------------------------------------------------------------------------------------------
int status = 0; //internet status; DISCONNECTED (1 = CONNECTED, 2 = SYNCED)
bool updateTime = true;
bool updatePeripheralTime = false;
bool timeUpdated = false;
bool wifiActive = false;
bool bleActive = false;
unsigned long timeEpoch = 0;

//----------------------------------------------------------------------------------------------------------------------
// I/O
// Sensors/actuators pins go here
//----------------------------------------------------------------------------------------------------------------------

int WIFI_LED_PIN = LED_BUILTIN;



//----------------------------------------------------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin( 9600 );
  while ( !Serial );  // turn off for standalone code running

  pinMode( WIFI_LED_PIN, OUTPUT );

  Serial.println( PROJECT_NAME );

  //============================================================================
  //Initial conditions  for IoT Variables
  //============================================================================

  led = true; //LED ON
  netState = 2; //NETWORKS OFF
  
}


void loop()
{
  //COMMENTED-OUT TIMING = CURRENTLY OFF
// currently 5s 
#define NTP_SYNC_INTERVAL 1 * 5 * 1000 
// needs to be 1000 ms
#define TASK_UPDATE_INTERVAL 1000 

  static unsigned long previousMillis = 0;
  static unsigned long ntpSyncPreviousMillis = 0;
  
  bleTask(); //controls BLE connection and data transmission
  wifiTask(); //controls Cloud connection and data transmission


  // ========================= Run your code here ===========================
  // Do not use delay
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > TASK_UPDATE_INTERVAL )
  {
    previousMillis = currentMillis;
    timeEpoch++;
    led = !led; //toggle value
    Serial.print("led Cloud Variable: ");
    Serial.println(led);
      
    //====================================================
    // CALL YOUR TASK 
    //====================================================
    
    delay(4000); //acting like the task (waiting, updating, calculating, etc.)

  }
  // ========================================================================
  
  currentMillis = millis();
  if ( currentMillis - ntpSyncPreviousMillis > NTP_SYNC_INTERVAL )
  {
    ntpSyncPreviousMillis = currentMillis;
    updateTime = true;
  }

  if ( timeUpdated )
  {
    timeUpdated = false;
    updatePeripheralTime = true;
  }

}

void wifiTask( void )
{
  //DO NOT EDIT.
  enum WIFI_STATE_TYPE { WIFI_STATE_OFF,
                         WIFI_STATE_CLOUD_PROPERTIES,
                         WIFI_STATE_CLOUD_CONNECT,
                         WIFI_STATE_CLOUD_UPDATE,
  
                         WIFI_STATE_END,
                         WIFI_STATE_RESTART = 255
                       };

  #define WIFI_CONNECT_TIMEOUT 10000
  #define WIFI_NTP_TIMEOUT 5000

  static int state = WIFI_STATE_OFF;
  static int wifiConnectTry = 0;
  static int ntpSendTry = 0;
  static int wifiStatus = WL_IDLE_STATUS;
  static unsigned long previousMillis = 0;

  // Serial.print("Update time: ");
  // Serial.println(updateTime);
  // Serial.print("bleActive: ");
  // Serial.println(bleActive);  
  switch ( state )
  {
    case WIFI_STATE_OFF:
      if ( ! bleActive )//updateTime && 
      {
        if (!Serial){ Serial.begin( 9600 );}
        wifiActive = true;
        updateTime = false;
        Serial.println( "WiFi active" );
        state++;
        break;
      }
      break;
    case WIFI_STATE_CLOUD_PROPERTIES:
      // Defined in thingProperties.h
      initProperties();
      Serial.println("Properties loaded.");
      state++;
      break;
    case WIFI_STATE_CLOUD_CONNECT:
      Serial.print("Connecting..");
      
      // unsigned long sm = millis();
      // Connect to Arduino IoT Cloud
      ArduinoCloud.begin(ArduinoIoTPreferredConnection);

      // establish callbacks for status tracking.
      ArduinoCloud.addCallback(ArduinoIoTCloudEvent::CONNECT, statusConnect);
      // ArduinoCloud.addCallback(ArduinoIoTCloudEvent::DISCONNECT, statusDisconnect);
      // ArduinoCloud.addCallback(ArduinoIoTCloudEvent::SYNC, statusSynched);
      
      
      // while (sm-millis()<WIFI_NTP_TIMEOUT){
      //   continue;//Wait 5 seconds for connection
      // }
      setDebugMessageLevel(2);
      ArduinoCloud.printDebugInfo();
      
      state++;
      digitalWrite( WIFI_LED_PIN, HIGH );
      netState = 1; //WIFI ON
      
      break;

      //TODO: make sure millis wait is needed
      if ( millis() - previousMillis < WIFI_CONNECT_TIMEOUT && wifiConnectTry > 0 )
      {
        // just continue with rest of program for now
        break;
      }
      // if ( wifiConnectTry > 10 )
      // {
      //   // could not connect, clear everything and try again later
      //   state = WIFI_STATE_RESTART;
      //   break;
      // }

      previousMillis = millis();
      wifiConnectTry++;
      Serial.print( "Try: " );
      Serial.print( wifiConnectTry );
      Serial.print( " Status: " );
      Serial.println( wifiStatus );
      break;
    case WIFI_STATE_CLOUD_UPDATE:
      Serial.println(ArduinoCloud.connected());
      if (ArduinoCloud.connected())
      {
          ArduinoCloud.update();

          //============================================================
          // WiFi function calls go here
          //============================================================

        state++;
        Serial.println( "Data transfer complete" );
        break;
      }

      if ( millis() - previousMillis < WIFI_NTP_TIMEOUT )
      {
        // just continue with rest of program for now
        break;
      }
      state = WIFI_STATE_END;
      break;
    case WIFI_STATE_END:
      state = WIFI_STATE_OFF;
      wifiConnectTry = 0;
      netState = 2; //OFF
      ArduinoIoTPreferredConnection.disconnect(); 
      WiFi.disconnect();
      WiFi.end();
      digitalWrite( WIFI_LED_PIN, LOW );
      wifiActive = false;
      Serial.println( "WiFi end" );
      Serial.end();
      break;
    default:
      state = WIFI_STATE_CLOUD_CONNECT;
      wifiConnectTry = 0;
      netState = 2; //OFF 
      ArduinoIoTPreferredConnection.disconnect(); 
      WiFi.disconnect();
      WiFi.end();
      digitalWrite( WIFI_LED_PIN, LOW );
      Serial.println( "WiFi restart" );
      Serial.end();
      break;
  }
}

void bleTask()
{
  enum BLE_STATE_TYPE { BLE_STATE_OFF,
                        BLE_STATE_BEGIN,
                        BLE_STATE_START_SCAN,
                        BLE_STATE_SCAN,
                        BLE_STATE_STOP_SCAN,
                        BLE_STATE_PERIPHERAL_CONNECT,
                        BLE_STATE_PERIPHERAL_DISCOVER,
                        BLE_STATE_PERIPHERAL_WRITE,

                        BLE_STATE_END,
                        BLE_STATE_RESTART = 255
                      };

#define BLE_SCAN_TIMEOUT 10000

  static int state = BLE_STATE_OFF;
  static unsigned long previousMillis = 0;
  static BLEDevice peripheral;

  switch ( state )
  {
    case BLE_STATE_OFF:
      if (! wifiActive ) // updatePeripheralTime && 
      {
        if (!Serial){ 
          Serial.begin( 9600 );
        }
        bleActive = true;
        updatePeripheralTime = false;
        Serial.println( "BLE active" );
        state++;
        break;
      }
      break;
    case BLE_STATE_BEGIN:
      Serial.println( "Create BLE object" );

      if ( !BLE.begin() )
      {
        Serial.println( "Starting BLE failed!" );
        break;
      }
      state++;
      break;
    case BLE_STATE_START_SCAN:
      Serial.println("Scan initiated.");
      BLE.scanForUuid( BLE_UUID_SERVICE_LED );
      previousMillis = millis();
      state++;
      break;
    case BLE_STATE_SCAN:
      if ( millis() - previousMillis > BLE_SCAN_TIMEOUT )
      {
        Serial.println("Couldn't find peripheral. Exiting BLE cycle.");
        BLE.stopScan();
        state = BLE_STATE_END;
        break;
      }

      peripheral = BLE.available();
      if ( !peripheral )
      {
        break;
      }
      if ( peripheral.localName() != LOCALNAME )
      {
        break;
      }
      state++;
      Serial.println( " perippheral found." );
      break;
    case BLE_STATE_STOP_SCAN:
      BLE.stopScan();
      state++;
      break;
    case BLE_STATE_PERIPHERAL_CONNECT:
      if ( !peripheral.connect() )
      {
        state = BLE_STATE_START_SCAN;
      }
      state++;
      break;
    case BLE_STATE_PERIPHERAL_DISCOVER:
      if ( !peripheral.discoverAttributes() )
      {
        peripheral.disconnect();
        state = BLE_STATE_START_SCAN;
      }
      state++;
      break;
    case BLE_STATE_PERIPHERAL_WRITE:
      {
        // BLECharacteristic battCharacteristic = peripheral.characteristic(BLE_UUID_BATTERY_MONITOR);
        BLECharacteristic ledCharacteristic = peripheral.characteristic(BLE_UUID_CHAR_SWITCH); 

        if ( ledCharacteristic) {

          if (led){
            ledCharacteristic.writeValue((byte)0x01); //turn LED on
            Serial.println("LED ON");
          }
          else{
            ledCharacteristic.writeValue((byte)0x00); //turn LED off   
            Serial.println("LED OFF");
          }

          Serial.println("BLE data transmission completed.");

        }
      }
      state++;
      break;
    case BLE_STATE_END:
      state = BLE_STATE_OFF;
      BLE.end();
      // Re-initialize the WiFi driver
      // This is currently necessary to switch from BLE to WiFi
      wiFiDrv.wifiDriverDeinit();
      wiFiDrv.wifiDriverInit();
      bleActive = false;
      updateTime = true; // to fix WiFi bug.
      Serial.println( "BLE end" );
      Serial.end();
      break;
    default:
      state = BLE_STATE_OFF;
      BLE.end();
      // Re-initialize the WiFi driver
      // This is currently necessary to switch from BLE to WiFi
      wiFiDrv.wifiDriverDeinit();
      wiFiDrv.wifiDriverInit();
      bleActive = false;
      updateTime = true; // to fix WiFi bug.
      Serial.println( "BLE end" );
      Serial.end();
      break;
  }
}

void loopAnalysis()
{
  static unsigned long previousMillis = 0;
  static unsigned long lastMillis = 0;
  static unsigned long minLoopTime = 0xFFFFFFFF;
  static unsigned long maxLoopTime = 0;
  static unsigned long loopCounter = 0;

#define LOOP_ANALYSIS_INTERVAL 1000

  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > LOOP_ANALYSIS_INTERVAL )
  {
    Serial.print( "Loops: " );
    Serial.print( loopCounter );
    Serial.print( " ( " );
    Serial.print( minLoopTime );
    Serial.print( " / " );
    Serial.print( maxLoopTime );
    Serial.println( " )" );
    previousMillis = currentMillis;
    loopCounter = 0;
    minLoopTime = 0xFFFFFFFF;
    maxLoopTime = 0;
  }
  loopCounter++;
  unsigned long loopTime = currentMillis - lastMillis;
  lastMillis = currentMillis;
  if ( loopTime < minLoopTime )
  {
    minLoopTime = loopTime;
  }
  if ( loopTime > maxLoopTime )
  {
    maxLoopTime = loopTime;
  }
}

/*
  Since Led is READ_WRITE variable, onLedChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLedChange()  {
  // Add your code here to act upon Led change
  analogWrite(LED_BUILTIN,led);
}

void statusConnect() {
  // update status to connected
  status = 1; //CONNECTED
  Serial.println("Successfully connected to Arduino IoT Cloud");
}