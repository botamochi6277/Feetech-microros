# Feetech microros

![badge](https://github.com/botamochi6277/Feetech_microros/actions/workflows/ci-platformio.yml/badge.svg)

Test to drive feetech ttl servo motors with sweeping.
M5Atom Lite controls [STS3215](https://akizukidenshi.com/catalog/g/gM-16312/) with [FE-URT-1](https://akizukidenshi.com/catalog/g/gM-16295/), interface board.

## Usage

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

## Test

```
ros2 topic pub feetech_state sensor_msgs/msg/JointState '{name:[ "joint01"], position:[0]}' --once
```

```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
