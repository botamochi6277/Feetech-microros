# Feetech microros

![badge](https://github.com/botamochi6277/Feetech-microros/actions/workflows/ci-platformio.yml/badge.svg)

Seeeduino XIAO control feetech ttl servo motors with micro-ROS interface.
The XIAO communicates with [STS3215](https://akizukidenshi.com/catalog/g/gM-16312/) using [FE-URT-1](https://akizukidenshi.com/catalog/g/gM-16295/), interface board. And the XIAO subscribes `feetech_state` topic as `sensor_msg/JointState` message.

## Build and Upload

1-Clone this repository:

```zsh
git clone  https://github.com/botamochi6277/Feetech-Sweep.git
```

2-Install SCServo library

```zsh
./download_scservo.sh
```

3-Build with PlatformIO

## Connection

**CAUTION**: FE-URT-1 would have wrong silk annotations about RXD and TXD.

| ROS2 |     |  XIAO   |     |  FE-URT-1  |
| :--: | :-: | :-----: | :-: | :--------: |
| USB  | --  |   USB   |     |            |
|      |     |   5V    | --  |     5V     |
|      |     |   GND   | --  |    GND     |
|      |     | D6 (RX) | --  | RXD (Silk) |
|      |     | D7 (TX) | --  | TXD (Silk) |

## Communication

ros2, run agent :

```zsh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
```

ros2, publish command example:

```zsh
ros2 topic pub feetech_state sensor_msgs/msg/JointState '{name:["joint_1","joint_2"], position:[0.75,0.75]}' --once
```
