#ifndef CONFIG_H
#define CONFIG_H



// WiFi configuration
#define WIFI_SSID       "U+NetEFDA"
#define WIFI_PASSWORD   "5D2A1A9EM#"
#define AGENT_IP        "192.168.219.107"
#define AGENT_PORT      8888


// -----------------------------
// Motor Driver (TB6612FNG) Pin Definitions
// -----------------------------
// Left Motor (Motor A)
#define LEFT_MOTOR_IN1    3
#define LEFT_MOTOR_IN2    10
#define LEFT_MOTOR_PWM    11

// Right Motor (Motor B)
#define RIGHT_MOTOR_IN1   8
#define RIGHT_MOTOR_IN2   7
#define RIGHT_MOTOR_PWM   6

// -----------------------------
// Encoder Pin Definitions
// -----------------------------
#define LEFT_ENCODER_A    13
#define LEFT_ENCODER_B    14
#define RIGHT_ENCODER_A   4
#define RIGHT_ENCODER_B   5

// -----------------------------
// Status LED Pin
// -----------------------------
#define LED_PIN           2

// -----------------------------
// Constants
// -----------------------------
#define ENCODER_CPR       360     // Encoder counts per revolution
#define RPM_TIMER_PERIOD  1000    // RPM calculation period in milliseconds

// -----------------------------
// PWM Parameters (using LEDC)
// -----------------------------
#define PWM_FREQ          1000    // 1 kHz
#define PWM_RESOLUTION    8       // 8-bit resolution

#endif // CONFIG_H

