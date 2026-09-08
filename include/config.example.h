#ifndef CONFIG_H
#define CONFIG_H

// Copy this file to include/config.h and fill in local values.
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define AGENT_IP        "192.168.1.10"
#define AGENT_PORT      8888

#define LEFT_MOTOR_IN1    3
#define LEFT_MOTOR_IN2    10
#define LEFT_MOTOR_PWM    11

#define RIGHT_MOTOR_IN1   8
#define RIGHT_MOTOR_IN2   7
#define RIGHT_MOTOR_PWM   6

#define LEFT_ENCODER_A    13
#define LEFT_ENCODER_B    14
#define RIGHT_ENCODER_A   4
#define RIGHT_ENCODER_B   5

#define LED_PIN           2

#define ENCODER_CPR       360
#define RPM_TIMER_PERIOD  1000

#define PWM_FREQ          1000
#define PWM_RESOLUTION    8

#endif
