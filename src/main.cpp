/*
  This example creates a BLE central that scans for a peripheral with a service containing a multi value characteristic.

  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 IoT board.

  This example code is in the public domain.
*/
#include <Arduino.h>
#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// HM10 BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_SENSOR_DATA_SERVICE              "ffe0"
#define BLE_UUID_MULTI_SENSOR_DATA                "ffe1"
#define BLE_POLL_INTERVALL 1000


#define BLE_SENSOR_DATA_BYTES 14
struct status_Item
  {
    char msg[BLE_SENSOR_DATA_BYTES];
  } __attribute__( ( packed ) );

union multi_sensor_data
{
  status_Item statusItem;
  uint8_t bytes[BLE_SENSOR_DATA_BYTES];

};


union multi_sensor_data multiSensorData;
byte statusData[BLE_SENSOR_DATA_BYTES]; // holder array for sensor data
bool newData = false;




/*
union multi_sensor_data
{
  struct __attribute__( ( packed ) )
  {
    int x;                    // 2 bytes  X Twist status confirmation. A float to int conversion to save 2 BLE bytes since we're only confirming changes.
    int z;                    // 2        Z Twist status confirmation. A float to int conversion to save 2 BLE bytes since we're only confirming changes.
  //  int mtr_pos_right;        // 2
  //  int mtr_pos_left;         // 2
  //  int mtr_speed_right;      // 2
  //  int mtr_speed_left;       // 2
  //  int sen_sonar_fwd;        // 2
  //  int sen_sonar_rear;       // 2
  //  int sen_ir_right;         // 2
  //  int sen_ir_left;          // 2
                              //====
                              // 20 Bytes  
  };
  uint8_t bytes[4];
};
union multi_sensor_data multiSensorData;
*/

struct  moveItem {
    float x;                    // 4 bytes Twist X
    float z;                    // 4       Twist Z 
    char cmd[3];                // 3       2 Chararter command variable
                                //====
                                // 11 Bytes
} __attribute__( ( packed ) );

// Union joins 11 Byte struct above to a uint8_t of 11 Bytes
union inputFromPC {
   moveItem moveData;
   uint8_t bytes[11];
};
 // this creates a working instance of the Union
 // elements in it are referred to as, e.g. inputData.moveData.x
inputFromPC inputData;

// byte data received
const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;

// Char data received
const byte numChars = BLE_SENSOR_DATA_BYTES;
char receivedChars[numChars];

void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rb;
   

    while ( newData == false ) {
        rb = Serial.read();

        if (recvInProgress == true) {
            if (rb != endMarker) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == startMarker) {
            recvInProgress = true;
        }
    }
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    
    while (newData == false) 
    {       
        
      for (byte n = 0; n < sizeof (multiSensorData.bytes) -1 ; n++) 
      {
        rc = multiSensorData.statusItem.msg[n];

        if (recvInProgress == true) 
        {
            if (rc != endMarker) 
            {
              receivedChars[ndx] = rc;
              ndx++;
                if (ndx >= numChars) 
                {
                    ndx = numChars - 1;
                }
            }
            else 
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
      }
    }  
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}

void receiveData() {
   /*
   if (Serial.available() < 11) {
     // error
     return;
   }
*/
   for (byte n = 0; n < 16; n++) {
      statusData[n] = multiSensorData.bytes[n];
   }
   // TODO check CRC
   for (byte n = 0; n < 14; n++) {
     multiSensorData.bytes [n] = statusData[n];
   }
   newData = true;
}


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

  while ( 1 ) // TBD ... need to add logic to leave
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
        // do that byte array trick.
        // receiveData();
        recvWithStartEndMarkers();
        

        if(newData){
          Serial.print("<BLE.Status.Msg::: ");
          // Serial.print( multiSensorData.statusItem.msg ); 
                   
          Serial.print(receivedChars); 
          //Serial.print( multiSensorData.z);
        /*
          Serial.print(" ");          
          Serial.print( multiSensorData.sen_sonar_fwd);
          Serial.print(" ");
          Serial.print( multiSensorData.sen_ir_right);
          Serial.print(" ");
          Serial.print( multiSensorData.sen_sonar_rear);
          Serial.print(" ");
          Serial.print( multiSensorData.sen_ir_left);
        */
          Serial.println(" :::BLE.Status.Msg>");
        }
      }

      if(multiSensorDataCharacteristic.canWrite() )
      {
        Serial.println( "BLE Characteristic is writable" );  
        // Send 11 Byte "bytes" which is  union of the 11 byte struct moveItem moveData
        inputData.moveData.x = 2.34; // This is where you would update the X value with current twist msg
        inputData.moveData.z = 1.00; // This is where you would update the Z value with current twist msg
        strcpy (inputData.moveData.cmd, "AB" ); // 2 Char command
        multiSensorDataCharacteristic.writeValue( inputData.bytes, sizeof inputData.bytes );
       // ~~TRY~~ THIS. WooOORKS! ^^^  
      
      }

    }
    //delay(100);
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

