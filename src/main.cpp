/*
  This example creates a BLE central that scans for a peripheral with a service containing a multi value characteristic.

  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 IoT board.

  This example code is in the public domain.
*/
#include <Arduino.h>
#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

// #define BLE_UUID_SENSOR_DATA_SERVICE              "2BEEF31A-B10D-271C-C9EA-35D865C1F48A"
// #define BLE_UUID_MULTI_SENSOR_DATA                "4664E7A1-5A13-BFFF-4636-7D0A4B16496C"

#define BLE_UUID_SENSOR_DATA_SERVICE              "ffe0"
#define BLE_UUID_MULTI_SENSOR_DATA                "ffe1"

#define NUMBER_OF_SENSORS 4
#define BLE_POLL_INTERVALL 5

/*


union multi_sensor_data multiSensorData;
*/


union multi_sensor_data
{
  struct __attribute__( ( packed ) )
  {
    char msg[5];
  };
  uint8_t bytes[5];
};
union multi_sensor_data multiSensorData;

struct moveItem {
    float x;                    // 4 bytes
    float z;                    // 4
    int testval;                // 2
    char debug[1];              // 1    
                                //====
                                // 11 Bytes
};

// Union joins 11 Byte struct above to a uint8_t of 11 Bytes
union inputFromPC {
   moveItem moveData;
   uint8_t bytes[11];
};
 // this creates a working instance of the Union
 // elements in it are referred to as, e.g. inputData.moveData.x
inputFromPC inputData;

bool explorePeripheral( BLEDevice peripheral )
{
  if ( !peripheral.connect() )
  {
    return false;
  }
  Serial.println( "BLE connected" );

  if ( !peripheral.discoverAttributes() )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE attributes discovered" );

  BLECharacteristic multiSensorDataCharacteristic = peripheral.characteristic( BLE_UUID_MULTI_SENSOR_DATA );
  if ( !multiSensorDataCharacteristic )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE characteristic found" );

  if ( !multiSensorDataCharacteristic.canSubscribe() )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE characteristic can subscribe" );

  if ( !multiSensorDataCharacteristic.subscribe() )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE characteristic subscribed" );

  while ( 1 ) // need to add logic to leave
  {
    
    static long previousMillis = 0;
    unsigned long currentMillis = millis();
    if ( currentMillis - previousMillis > BLE_POLL_INTERVALL )
    {
      BLE.poll();

      if ( multiSensorDataCharacteristic.valueUpdated() )
      {
        Serial.println( "BLE new data" );
        /// this is where things get different.
       
        multiSensorDataCharacteristic.readValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
        
          Serial.println( multiSensorData.msg ); 
      }
      if(multiSensorDataCharacteristic.canWrite() ){
        Serial.println( "BLE Characteristic is writable" );  
        // Send 11 Byte "bytes" which is  union of the 11 byte struct moveItem moveData
        inputData.moveData.x = 2.34;
        inputData.moveData.z = 1.00;
        inputData.moveData.testval =1234;
        strcpy (inputData.moveData.debug, "A" );
        multiSensorDataCharacteristic.writeValue( inputData.bytes, sizeof inputData.bytes );
        delay(100);
       // TRY THIS? ^^^  
       

      }

    }
  }

   peripheral.disconnect();
  
  return true;
}


void setup()
{
  
  Serial.begin( 57600 );
  while ( !Serial );

  BLE.begin();

  BLE.scanForUuid( BLE_UUID_SENSOR_DATA_SERVICE );
}


void loop()
{
#define UPDATE_INTERVALL 10
  static long previousMillis = 0;

  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > UPDATE_INTERVALL )
  {
    previousMillis = currentMillis;

    BLEDevice peripheral = BLE.available();

    if ( peripheral )
    {
      if ( peripheral.localName() != "HMSoft" )
      {
        return;
      }

      BLE.stopScan();

      explorePeripheral( peripheral );

      BLE.scanForUuid( BLE_UUID_SENSOR_DATA_SERVICE );
    }
  }
}

