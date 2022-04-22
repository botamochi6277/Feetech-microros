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

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#define LED_HEARTBEAT_PIN 13
#define LED_SUBSCRIPTION_PIN 0
#define LED_ERROR_PIN 1

SMS_STS feetech; // Feetech Bridge

// node
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
// rcl_publisher_t publisher;
// std_msgs__msg__String msg_string;
// rclc_executor_t executor_pub;
// rcl_timer_t timer;

// subscriber
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState *msg_jointstate;
rclc_executor_t executor_sub;

sensor_msgs__msg__JointState *create_joint_states_message(size_t num_reserve = 5)
{
  sensor_msgs__msg__JointState *msg = sensor_msgs__msg__JointState__create();
  rosidl_runtime_c__String__assign(&msg->header.frame_id, "world");
  rosidl_runtime_c__String__Sequence__init(&msg->name, num_reserve); // Number of join in arm

  for (size_t i = 0; i < num_reserve; i++)
  {
    String s = "my_awesome_joint_" + String(i);
    rosidl_runtime_c__String__assign(&msg->name.data[i], s.c_str());
  }
  rosidl_runtime_c__float64__Sequence__init(&msg->position, 5);
  rosidl_runtime_c__float64__Sequence__init(&msg->velocity, 5);
  rosidl_runtime_c__float64__Sequence__init(&msg->effort, 5);
  return msg;
}

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
    digitalWrite(LED_ERROR_PIN, !digitalRead(LED_ERROR_PIN));
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
  digitalWrite(LED_SUBSCRIPTION_PIN, !digitalRead(LED_SUBSCRIPTION_PIN));
  const sensor_msgs__msg__JointState *msg_jointstate = (const sensor_msgs__msg__JointState *)msgin;

  if (msg_jointstate->name.size != msg_jointstate->position.size)
  {
    // array size is not matched
    // msg_string.data.data = "array size unmatch";
    return;
  }

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
    String s_no = name.substring(loc + 1);
    unsigned char id = s_no.toInt();
    feetechWrite(feetech, id, angle);
    // debug_text += "id: " + String(id) + ", ";
    // debug_text += "position: " + String(angle, 2) + "\n";
  }
  feetech.RegWriteAction();

  // char buff[128];
  // debug_text.toCharArray(buff, 128);
  // msg_string.data.data = buff;

  // RCSOFTCHECK(rcl_publish(&publisher, &msg_string, NULL));
}

void setup()
{
  pinMode(LED_HEARTBEAT_PIN, OUTPUT);
  pinMode(LED_ERROR_PIN, OUTPUT);
  pinMode(LED_SUBSCRIPTION_PIN, OUTPUT);
  digitalWrite(LED_ERROR_PIN, HIGH);
  digitalWrite(LED_SUBSCRIPTION_PIN, HIGH);

  Serial1.begin(1000000); // serial for servos
  feetech.pSerial = &Serial1;

  delay(1000); // waiting for servo connection
  // wakeup sweep
  float wu_angles[] = {
      0.25f * PI_F, // neutral
      0.35f * PI_F,
      0.15f * PI_F,
      0.25f * PI_F,
  };
  int ids[] = {1, 2, 3, 4};
  wakeup_sweep(feetech, wu_angles, 4, ids, 4);

  // micro ros setup
  set_microros_transports();

  delay(2000); // wait for micro-ros connection
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_jointstate_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "feetech_state"));

  msg_jointstate = create_joint_states_message(16); // assign memory

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, msg_jointstate, &subscription_callback, ON_NEW_DATA));
}

void loop()
{
  delay(100);
  // heartbeat
  digitalWrite(LED_HEARTBEAT_PIN, !digitalRead(LED_HEARTBEAT_PIN));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}