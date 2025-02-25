#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"
#include "MicroROSNode.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting motor control with micro-ROS over USB...");

  // Set status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initialize Motor Control module
  MotorControl::begin();

  // Initialize micro-ROS node
  MicroROSNode::begin();
}

void loop() {
  MicroROSNode::spin();
  delay(10);
}
