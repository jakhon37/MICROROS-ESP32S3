#ifndef MICRO_ROS_NODE_H
#define MICRO_ROS_NODE_H

#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"

#if defined(USE_USB)
  #include <micro_ros_platformio.h>
#else
  #error "This project is configured for USB transport only."
#endif

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

class MicroROSNode {
public:
  static void begin();
  static void spin();

private:
  static rcl_node_t node;
  static rcl_publisher_t left_rpm_pub;
  static rcl_publisher_t right_rpm_pub;
  static rcl_subscription_t cmd_vel_sub;
  static rcl_timer_t timer;
  static rclc_executor_t executor;
  static rclc_support_t support;
  static rcl_allocator_t allocator;

  static std_msgs__msg__Float32 left_rpm_msg;
  static std_msgs__msg__Float32 right_rpm_msg;
  static geometry_msgs__msg__Twist cmd_vel_msg;

  static void cmdVelCallback(const void *msgin);
  static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif // MICRO_ROS_NODE_H
