broken code from here but an example to invesigate.
https://forum.arduino.cc/t/how-to-get-an-answer-from-peripheral-and-standalone-hm-10/676935


```cpp
#include <ArduinoBLE.h>

//Device specifications
#define HM10_CONSOLE_SERVICE_UUID          "FFE0"                   //Advertised Service of HM-10
#define HM10_CONSOLE_CHARACTERISTIC_UUID   "FFE1"                   //Characteristic of HM-10 - Communication is transmitted here
#define HM10_CONSOLE_DESCRIPTOR_UUID       "2902"                   //CCCD of HM-10 - Subscribe to notifications here

//Communication variables
char request[] = {0x41,0x54,0x2b,0x43,0x4f,0x4c,0x3f,0x3f};         //request string - HEX->String: AT+COL??
const unsigned int message_size = 20;                               //Size of message
char message[message_size];                                         //message string - what will be received
const unsigned int counterReq_limit = 5;                            //Amount of readValue() before writeValue() occurs + 1 when sending a command

/*
//LEDs switching variables
const unsigned int setLed_size = 8;
char set_ledGon[] = {0x41,0x54,0x2b,0x50,0x49,0x4f,0x42,0x31};      //"AT+PIOB1" - Set PIO11:1 (Green on)
char set_ledRon[] = {0x41,0x54,0x2b,0x50,0x49,0x4f,0x37,0x31};      //"AT+PIO71" - Set PIO7:1 (Red on)
char set_ledGoff[] = {0x41,0x54,0x2b,0x50,0x49,0x4f,0x42,0x30};     //"AT+PIOB0" - Set PIO11:0 (Green off)
char set_ledRoff[] = {0x41,0x54,0x2b,0x50,0x49,0x4f,0x37,0x30};     //"AT+PIO70" - Set PIO7:0 (Red off)
*/

//Functions
void initializeBLE();
void scanBLE(char[]);
void discoverBLEAttributes(BLEDevice);
void interactBLECharacteristic(BLEDevice,char[]);
void checkAllBLECharacteristicDescriptors(BLECharacteristic);
void checkSpecificBLECharacteristicDescriptor(BLECharacteristic,char[]);

void setup() {
  // put your setup code here, to run once:
  // initialize serial
  initializeBLE();
}

void loop() {
  // put your main code here, to run repeatedly:
  scanBLE(peripheralMAC);
}

void initializeBLE(){
}

void scanBLE(char MAC[]){
//Scan for peripherals and connect to them
}

void discoverBLEAttributes(BLEDevice peripheral){
//Discover several attributes, not here to save space
}

void interactBLECharacteristic(BLEDevice peripheral, char CharacteristicsUUID[]){
  Serial.print("Interact with Characteristic: ");
  Serial.println(CharacteristicsUUID);
  BLECharacteristic bleCharacteristic = peripheral.characteristic(CharacteristicsUUID);
  if(bleCharacteristic){
    Serial.println("Check if device is subscribable");
    if(!bleCharacteristic.canSubscribe()){
      Serial.println("Characteristic not subscribable.");
    }else Serial.println("Characteristic subscribable.");

    checkAllBLECharacteristicDescriptors(bleCharacteristic);                                       //Check all available descriptors

    checkSpecificBLECharacteristicDescriptor(bleCharacteristic,HM10_CONSOLE_DESCRIPTOR_UUID);      //print the value of descriptor 2902
    
    Serial.println("Subscribe to characteristic");
    if(bleCharacteristic.subscribe()){
      Serial.println("Successfully sent subscription request.");
    }else Serial.println("Failed sending subscription request.");

    checkSpecificBLECharacteristicDescriptor(bleCharacteristic,HM10_CONSOLE_DESCRIPTOR_UUID);      //print the value of descriptor 2902 -> if it's 1 subscription was successful
    
    Serial.println("Check if characteristic is readable:");
    if(bleCharacteristic.canRead()){
      Serial.println("Characteristic is readable");

      //AT Commands are now send since everything is set up
      int counter = counterReq_limit;                                                 //Setup a counter, every 5 iterations a request is send to the HM-10, every iteration the HM-10's answer is printed
      while(Serial && bleCharacteristic && peripheral.connected()){
        if(counter >= counterReq_limit){
          counter = 0;
          Serial.println("Sending 'AT+COL??'");
          Serial.println("Received:");
          bleCharacteristic.writeValue(request);                                      //Send the request "AT+COL??"
          //while(!bleCharacteristic.valueUpdated());                                 //Wait for a new notification -> turned off because it's showing the same result as when reading whitout waiting
        }   
        if(bleCharacteristic.valueUpdated()) Serial.println("Value updated!");        //Inform about a notification
        bleCharacteristic.readValue(message,message_size);                            //Read the answer into the buffer
        Serial.println(message);                                                      //Print the buffer
        counter++;
      }
      /*
      //LED Switching on HM-10
      Serial.println("Switching LEDs");
      while(peripheral.connected() && Serial && bleCharacteristic){
        if(bleCharacteristic.writeValue(set_ledRon,setLed_size)){
          Serial.println("Successfully set Red");
          bleCharacteristic.writeValue(set_ledGoff,setLed_size);
          delay(500);
        }else Serial.println("Error setting Red");
        if(bleCharacteristic.writeValue(set_ledGon,setLed_size)){
          Serial.println("Successfully set Green");
          bleCharacteristic.writeValue(set_ledRoff,setLed_size);
          delay(500); 
        }else Serial.println("Error setting Green");
      }
      */
    }else Serial.println("Characteristic is not readable");
  }
}

void checkAllBLECharacteristicDescriptors(BLECharacteristic bleCharacteristic){
  //Loop the descriptors of the characteristic and explore each, also print out the values
}

void checkSpecificBLECharacteristicDescriptor(BLECharacteristic bleCharacteristic,char descriptorUUID[]){
  //Get a descriptor value for a descriptor with UUID = descriptorUUID 
}
```