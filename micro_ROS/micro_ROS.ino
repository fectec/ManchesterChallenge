// Include libraries to be used

#include <micro_ros_arduino.h>      // micro-ROS for ESP32
#include <rcl/rcl.h>                // Core ROS 2 Client Library
#include <rcl/error_handling.h>     // Error handling utilities
#include <rclc/rclc.h>              // Micro-ROS Client Library
#include <rclc/executor.h>          // Micro-ROS Executor
#include <std_msgs/msg/int32.h>     // Message type for encoder count
#include <std_msgs/msg/float32.h>   // Message type for PWM signal
#include <stdio.h>                  // Standard I/O library
#include <math.h>                   // Standard math library

// Encoder pins and count variable
#define ENCODER_A 35
#define ENCODER_B 34 
volatile int32_t encoder_count = 0;

// PWM and H-Bridge control definitions
#define PWM_PIN   4    // PWM output to motor
#define DIR_IN1   18   // Motor direction pin 1
#define DIR_IN2   15   // Motor direction pin 2
#define PWM_FRQ   980  // PWM frequency
#define PWM_RES   8    // PWM resolution (8 bits)
#define PWM_CHNL  0    // PWM channel

// Nodes
rcl_node_t motor_node;                // ROS 2 Node running on the MCU

// Instantiate executor and its support classes
rclc_executor_t executor;             // Manages task execution (timers, callbacks, etc.)
rclc_support_t support;               // Handles initialization & communication setup
rcl_allocator_t allocator;            // Manages memory allocation

// Publishers
rcl_publisher_t encoder_publisher;    // ROS 2 Publisher for sending encoder data

// Subscribers 
rcl_subscription_t pwm_subscriber;    // ROS 2 Subscriber for receiving PWM data

// Timers
rcl_timer_t timer;                    // Timer to execute functions at intervals

// Messages
std_msgs__msg__Int32 encoder_msg;
std_msgs__msg__Float32 pwm_msg;

// MACROS
// Executes a function and calls error_loop() if it fails
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Executes function but ignores failures
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error function
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Encoder Interrupt Service Routines (ISRs)

void IRAM_ATTR encoder_isr_A() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  encoder_count += (A == B) ? 1 : -1;  // Clockwise or counter-clockwise
}

void IRAM_ATTR encoder_isr_B() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  encoder_count += (A != B) ? 1 : -1;  // Clockwise or counter-clockwise
}

// Timer callback function (publishes encoder data)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);                                           // Prevents compiler warnings about an unused parameter

  if (timer != NULL) {                                                   // Ensures the timer event is valid before executing actions
    // Read the current encoder count (use a temporary variable to avoid issues with the interrupt)
    int32_t current_count;
    noInterrupts();
    current_count = encoder_count;
    interrupts();

    // Update the message with the current encoder count
    encoder_msg.data = current_count;

    // Publish the message         
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));   // Publishes msg to the ROS 2 topic
  }
}

// PWM subscription callback function (handles motor control)
void pwm_subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float received_value = constrain(msg->data, -1.0, 1.0);  // Limit range
  
  // Convert -1 to 1 range into a valid PWM duty cycle (0-255)
  int pwm_value = (int)(fabs(received_value) * ((1 << PWM_RES) - 1));

  // Motor control logic
  if (received_value > 0) {
    digitalWrite(DIR_IN1, HIGH);
    digitalWrite(DIR_IN2, LOW);
  } else if (received_value < 0) {
    digitalWrite(DIR_IN1, LOW);
    digitalWrite(DIR_IN2, HIGH);
  } else {  
    digitalWrite(DIR_IN1, LOW);
    digitalWrite(DIR_IN2, LOW);
  }

  ledcWrite(PWM_CHNL, pwm_value);
}

void setup() {
  // Setup microcontroller pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(DIR_IN1, OUTPUT);
  pinMode(DIR_IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Attach interrupts to encoder pins A & B for quadrature detection
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isr_B, CHANGE);

  // Setup PWM
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  // Initializes communication between ESP32 and the ROS 2 Agent (serial)
  set_microros_transports();

  // Connection delay
  delay(2000);

  // Initializes memory allocation for micro-ROS operations
  allocator = rcl_get_default_allocator();

  // Creates a ROS 2 support structure to manage the execution context
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&motor_node, "motor", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_encoder_data"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &pwm_subscriber,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm"));

  // Create timer
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initializes the micro-ROS executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Register subscriptor & timer
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &pwm_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize message
  encoder_msg.data = 0;
}

void loop() {
  // Executor spin
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}