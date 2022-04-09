#include "wifi.h" // add your wifi credentials and ROS agent IP here.

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/twist.h> 

/* SERIAL TRANSFER */
#include <SerialTransfer.h> // https://github.com/PowerBroker2/SerialTransfer
SerialTransfer myTransfer;
struct STRUCT {
  char z;
  float y;
} testStruct;
char arr[] = "hello";

/* END SERIAL TRANSFER */


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif


std_msgs__msg__Int32 msg;
rcl_subscription_t subscriber;  // Subscriber  
geometry_msgs__msg__Twist twistmsg;  // Twist Msg 
rcl_publisher_t publisher;      // Publisher
rclc_support_t support;         // Support 
rcl_allocator_t allocator;      //Allocator
rclc_executor_t executor;       // Executor
rcl_node_t node;                // Node 



#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/* dtostrf - Emulation for dtostrf function from avr-libc 
https://github.com/arduino/Arduino/blob/a2e7413d229812ff123cb8864747558b270498f1/hardware/arduino/sam/cores/arduino/avr/dtostrf.c
*/
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}


/*
START EXPERINMENT
*/

// Global Twist Vars 
float demandx=0;
float demandz=0;

// callback function for cmd_vel topic
void cmd_vel_cb( const void *msgin){
  String output;

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    // if velocity in x direction is 0 turn off LED, if 1 turn on LED
    // remove this after testing.
    if((msg->linear.x == 0)){
      digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
      delay(500);
      digitalWrite(LED_PIN, HIGH);
  } 

  demandx = msg->linear.x;
  demandz = msg->angular.z;
 
  char linearX[32];
  char angularZ[32];
  
  dtostrf(msg->linear.x, 20, 16, linearX );
  dtostrf(msg->angular.z, 20, 16, angularZ); 
  output = "X:" + String(linearX)  + "\nZ:" + String(angularZ); 
   
   if (true){ // placeholder for future test.
      Serial.println(output);           
   }

}


/*
END EXPERINMENT
*/

void setup() {
  // Setup UART 
  Serial.begin(57600);
  
 /* SerialTransfer.h Test */ 
  myTransfer.begin(Serial); 
  testStruct.z = '$'; 
  testStruct.y = 4.5; 
/* End Test */

  // set_microros_transports();
  set_microros_wifi_transports(SSID, WIFIPASSWD, AGENTIP, 8888); // SSID WIFIPASSWD AGENTIP. All defined in wifi.h. Transport protocol of this function defaults to UDP4

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  // Initialize micro-ROS allocator
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "micro_ros_arduino_twist_subscriber"));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "bot_status"));  
    msg.data = 0;


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_cb, ON_NEW_DATA));
 
}

void loop() {
 msg.data++; // Useless Example. increment the msg.data 
 RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL)); // publish data.

  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Run Executor.

/* SerialTransfer.h Test */ 
  // use this variable to keep track of how many
  // bytes we're stuffing in the transmit buffer
  uint16_t sendSize = 0;
  ///////////////////////////////////////// Stuff buffer with struct
  sendSize = myTransfer.txObj(testStruct, sendSize);
  ///////////////////////////////////////// Stuff buffer with array
  sendSize = myTransfer.txObj(arr, sendSize);
  ///////////////////////////////////////// Send buffer
  myTransfer.sendData(sendSize);
/* End SerialTransfer.h Test */ 

}
