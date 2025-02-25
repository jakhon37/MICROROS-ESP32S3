#include "MicroROSNode.h"
#include "config.h"
#include "MotorControl.h"

#if defined(USE_USB)
  #include <micro_ros_platformio.h>
#endif

// Static member definitions
rcl_node_t MicroROSNode::node;
rcl_publisher_t MicroROSNode::left_rpm_pub;
rcl_publisher_t MicroROSNode::right_rpm_pub;
rcl_subscription_t MicroROSNode::cmd_vel_sub;
rcl_timer_t MicroROSNode::timer;
rclc_executor_t MicroROSNode::executor;
rclc_support_t MicroROSNode::support;
rcl_allocator_t MicroROSNode::allocator;

std_msgs__msg__Float32 MicroROSNode::left_rpm_msg;
std_msgs__msg__Float32 MicroROSNode::right_rpm_msg;
geometry_msgs__msg__Twist MicroROSNode::cmd_vel_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}; }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}; }

void MicroROSNode::begin() {
  // Use USB serial transport for micro-ROS
  set_microros_serial_transports(Serial);
  Serial.println("Using USB serial transport for micro-ROS.");

  // Get default allocator and initialize support structure
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "motor_control_node", "", &support));

  // Create publishers for left and right motor RPM
  RCCHECK(rclc_publisher_init_default(
    &left_rpm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "left_motor_rpm"));
  RCCHECK(rclc_publisher_init_default(
    &right_rpm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "right_motor_rpm"));

  // Create subscription for cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create timer for RPM calculation & publishing
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(RPM_TIMER_PERIOD),
    timerCallback));

  // Create executor and add subscription and timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize RPM messages
  left_rpm_msg.data = 0.0;
  right_rpm_msg.data = 0.0;
}

void MicroROSNode::spin() {
  // Process micro-ROS executor events
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void MicroROSNode::cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;
  float leftSpeed = constrain(linear - angular, -1.0, 1.0);
  float rightSpeed = constrain(linear + angular, -1.0, 1.0);
  MotorControl::setMotorSpeeds(leftSpeed, rightSpeed);
}

void MicroROSNode::timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  // Get RPM values from MotorControl module
  left_rpm_msg.data = MotorControl::getLeftRPM();
  right_rpm_msg.data = MotorControl::getRightRPM();

  // Publish the RPM values
  RCSOFTCHECK(rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL));
  RCSOFTCHECK(rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL));

  // Optional: print to serial for debugging
  Serial.print("Left RPM: ");
  Serial.print(left_rpm_msg.data, 2);
  Serial.print("  Right RPM: ");
  Serial.println(right_rpm_msg.data, 2);
}
