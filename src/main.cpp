#include <Arduino.h>

#include "SCServo.h"
#include "feetech_utils.hpp"

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#define LED_PIN 13

SMS_STS st; // Feetech Bridge

// node
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// subscriber
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg_jointstate;
rclc_executor_t executor_sub;
rcl_timer_t timer;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
/**
 * @brief loop to indicate error with blinking LED
 *
 */
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL)
//   {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg_heartbeat, NULL));
//     msg_heartbeat.data++;
//   }
// }

/**
 * @brief subscription callback executed at receiving a message
 *
 * @param msgin
 */
void subscription_callback(const void *msgin)
{
  const sensor_msgs__msg__JointState *msg_jointstate = (const sensor_msgs__msg__JointState *)msgin;

  for (size_t i = 0; i < msg_jointstate->position.size; i++)
  {
    // example name: "joint_001"
    String name = String(msg_jointstate->name.data[i].data);
    float angle = msg_jointstate->position.data[i];
    int loc = name.lastIndexOf('_');
    if (loc < 0)
    {
      continue;
    }
    String s_no = name.substring(loc);
    unsigned char id = s_no.toInt();
    feetechWrite(st, id, angle);
  }
  st.RegWriteAction();
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  // Serial1.begin(1000000, SERIAL_8N1, RX1, TX1);
  Serial1.begin(1000000); // serial for servos
  st.pSerial = &Serial1;

  // debug print before uros comm.
  // Serial.begin(9600);

  // Serial.end();

  delay(1000); // waiting for servo connection
  // wakeup sweep
  float wu_angles[] = {
      0.25f * PI_F, // neutral
      0.35f * PI_F,
      0.15f * PI_F,
      0.25f * PI_F,
  };
  int ids[] = {1, 2, 3, 4};
  wakeup_sweep(st, wu_angles, 4, ids, 4);

  set_microros_transports();
  // (todo) waiting for connection
  delay(2000); // wait for uros connection
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_feetech_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "feetech_state"));
  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_jointstate, &subscription_callback, ON_NEW_DATA));
}

void loop()
{
  delay(100);
  // heartbeat
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}