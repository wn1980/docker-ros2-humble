#include <WiFiManager.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 12

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);

  // Create WiFiManager object
  WiFiManager wfm;

  // Supress Debug information
  //wfm.setDebugOutput(false);

  // Remove any previous network settings
  wfm.resetSettings();

  // Add custom parameter
  WiFiManagerParameter agent_ip_text_box("agent_ip", "Agent IP", "192.168.1.166", 15);
  WiFiManagerParameter agent_port_text_box("agent_port", "Port", "8888", 5);

  wfm.addParameter(&agent_ip_text_box);
  wfm.addParameter(&agent_port_text_box);

  if (!wfm.autoConnect()) {
    // Did not connect, print error message
    Serial.println("failed to connect and hit timeout");

    // Reset and try again
    ESP.restart();
    delay(1000);
  }

  // Connected!
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Print custom text box value to serial monitor
  Serial.print("Agent IP: ");
  Serial.println(agent_ip_text_box.getValue());
  Serial.print("Agent Port: ");
  Serial.println(agent_port_text_box.getValue());

  //Serial.println(WiFi.SSID());
  //Serial.println(WiFi.psk());
  
  char* ssid = (char*)WiFi.SSID().c_str();
  char* passwd = (char*)WiFi.psk().c_str();  
  char* agent_ip = (char*)agent_ip_text_box.getValue();
  uint agent_port = atoi(agent_port_text_box.getValue());

  //Serial.println(String(ssid));
  //Serial.println(String(passwd));
  //Serial.println(String(agent_ip));
  //Serial.println(String(agent_port));
  
  //WiFi.disconnect();
  
  //=============== micro-ROS setup ===============
  //set_microros_wifi_transports("***", "***", "192.168.1.166", 8888);
  set_microros_wifi_transports(ssid, passwd, agent_ip, agent_port);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize and modify options (Set ROS_DOMAIN_ID)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 50);

  //create init_options
  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "topic_name"));

  msg.data = 0;
}

void loop() {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
}
