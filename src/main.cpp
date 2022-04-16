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

SMS_STS st;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
// subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_led;

sensor_msgs__msg__JointState msg_jointstate;

rclc_executor_t executor_sub;
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
  // (condition) ? (true exec):(false exec)
  // digitalWrite(LED_PIN, (msg_led->data == 0) ? LOW : HIGH);

  for (size_t i = 0; i < msg_jointstate->position.size; i++)
  {
    float angle = msg_jointstate->position.data[i];
    feetechWrite(st, i, angle);
  }
  st.RegWriteAction();
}

void setup()
{
  // Serial1.begin(1000000, SERIAL_8N1, RX1, TX1);
  Serial1.begin(1000000); // serial for servos
  st.pSerial = &Serial1;
  delay(1000);
  // wakeup sweep
  float wu_angles[] = {
      0.25f * PI_F, // neutral
      0.35f * PI_F,
      0.15f * PI_F,
      0.25f * PI_F,
  };
  for (size_t j = 0; j < 4; j++)
  {
    for (size_t i = 0; i < 4; i++)
    {
      feetechWrite(st, i, wu_angles[j]);
    }
    st.RegWriteAction();
    delay(1000);
  }

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_feetech_node", "", &support));

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create subscriber
  // const char topic_name_led[] = "xiao_led_state";
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "feetech_state"));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_led, &subscription_callback, ON_NEW_DATA));
}

void loop()
{
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}