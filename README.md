# MICROROS-ESP32S3

micro-ROS firmware for a two-motor differential drive on an ESP32-S3. It takes `cmd_vel` from ROS 2 over USB serial, drives the motors, and publishes wheel RPM.

This is an earlier experiment. [autoJetsonBot](https://github.com/jakhon37/autoJetsonBot) now uses [MOTOR-ESP32S3](https://github.com/jakhon37/MOTOR-ESP32S3), a serial-bridge controller with PID.

## ROS 2 interface

Node: `motor_control_node` (ROS 2 Humble, serial transport).

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | Subscribed | Uses `linear.x` and `angular.z` |
| `left_motor_rpm` | `std_msgs/Float32` | Published | Every 1 s |
| `right_motor_rpm` | `std_msgs/Float32` | Published | Every 1 s |

Mixing is `left = linear.x - angular.z` and `right = linear.x + angular.z`, clamped to ±1 and applied as PWM duty.

## Hardware

An ESP32-S3 DevKitC-1, a dual H-bridge with IN1, IN2 and PWM inputs, and two quadrature encoders. PWM runs at 1 kHz with 8-bit resolution. Encoders are set to 360 counts per revolution.

| Signal | Left | Right |
|---|---|---|
| IN1 | GPIO 3 | GPIO 8 |
| IN2 | GPIO 10 | GPIO 7 |
| PWM | GPIO 11 | GPIO 6 |
| Encoder A | GPIO 13 | GPIO 4 |
| Encoder B | GPIO 14 | GPIO 5 |

The status LED is on GPIO 2. Change any of these in `include/config.h`.

## Build and run

`include/config.h` is gitignored, so create it first. With USB transport the WiFi and agent fields are not used.

```bash
cp include/config.example.h include/config.h
pio run -t upload
```

Start the agent on the ROS 2 machine. Your serial port may differ.

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Then drive the motors and read RPM.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
ros2 topic echo /left_motor_rpm
```

## Layout

```
src/main.cpp        setup and spin loop
lib/MicroROSNode/   node, publishers, cmd_vel subscriber, timer
lib/MotorControl/   PWM output, encoder interrupts, RPM calculation
include/            config.example.h
```

## Known limitations

- **Open loop.** There is no PID. Encoders are only used to report RPM.
- **No command timeout.** Motors keep the last `cmd_vel` if the agent or link is lost.
- **RPM timing.** Left and right readings share one timestamp, so the right value can spike if the millisecond counter ticks between the two calls.
- **Startup failures.** A failed micro-ROS init step halts the firmware in a loop.

## License

MIT
