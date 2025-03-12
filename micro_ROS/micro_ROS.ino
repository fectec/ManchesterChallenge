// Use esp32 by Espressif Systems Library 2.0.17 version

// ===== LIBRARY INCLUDES =====
#include <micro_ros_arduino.h>          // micro-ROS for ESP32
#include <rcl/rcl.h>                    // Core ROS 2 Client Library
#include <rcl/error_handling.h>         // Error handling utilities
#include <rclc/rclc.h>                  // micro-ROS Client Library
#include <rclc/executor.h>              // micro-ROS Executor
#include <std_msgs/msg/int32.h>         // Message type for encoder count
#include <std_msgs/msg/float32.h>       // Message type for float values
#include <rmw_microros/rmw_microros.h>  // For QoS settings
#include <stdio.h>                      // Standard I/O library
#include <math.h>                       // Standard math library

// ===== TIMING CONFIGURATION =====
// All timing values
#define CONTROL_LOOP_FREQ_HZ     50     // Control loop frequency
#define CONTROL_LOOP_PERIOD_MS   (1000 / CONTROL_LOOP_FREQ_HZ)  // Control loop period
#define EXECUTOR_SPIN_TIME_MS    1      // Time for executor to process messages
#define AGENT_CHECK_INTERVAL_MS  500    // How often to check for agent in WAITING_AGENT state
#define PING_CHECK_INTERVAL_MS   200    // How often to ping agent in CONNECTED state
#define PING_TIMEOUT_MS          100    // How long to wait for ping response
#define ENCODER_TIMEOUT_US       10000  // Timeout for encoder pulses

// ===== HARDWARE DEFINITIONS =====
#define LED_DEBUG_PIN 2

// Encoder
#define ENCODER_A 32              // Encoder channel A pin
#define ENCODER_B 33              // Encoder channel B pin
#define ENCODER_RESOLUTION 12.0   // Counts per motor shaft rotation 
#define ENCODER_GEAR_RATIO 34.0   // Motor shaft rotations per output shaft rotation      

// H-Bridge (Motor Driver)
#define PWM_PIN   4               // PWM output pin
#define PWM_FRQ   100             // PWM frequency
#define PWM_RES   8               // PWM resolution (8 bits)
#define PWM_CHNL  0               // PWM channel
#define PWM_IN1   18              // Motor direction pin 1
#define PWM_IN2   15              // Motor direction pin 2
#define PWM_MAX   255             // Maximum PWM value

// ===== HELPER MACROS =====
// Executes a function and returns false if it fails
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes function but ignores failures
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile uint32_t init = -1; \
  if (init == -1) {init = uxr_millis();} \
  if (uxr_millis() - init > MS) {X; init = uxr_millis();} \
} while(0)

// ===== STATE MACHINE DEFINITIONS =====
// micro-ROS connection state machine
enum states {
  WAITING_AGENT,        // Waiting for agent connection
  AGENT_AVAILABLE,      // Agent is available but not connected
  AGENT_CONNECTED,      // Agent is connected and operational
  AGENT_DISCONNECTED    // Agent was connected but disconnected
} state;

// ===== ROS 2 ENTITIES =====
// Node
rcl_node_t motor_node;                  // ROS 2 Node running on the MCU

// Allocator
rcl_allocator_t allocator;              // Manages memory allocation

// Support
rclc_support_t support;                 // Handles initialization & communication setup

// Executor
rclc_executor_t executor;               // Manages task execution (timers, callbacks, etc.)

// Publishers
rcl_publisher_t motor_output_publisher;    // Publishes actual motor angular velocity
rcl_publisher_t error_publisher;           // Publishes error
rcl_publisher_t motor_input_publisher;     // Publishes control signal to motor
rcl_publisher_t encoder_count_publisher;   // Publishes raw encoder count

// Subscribers
rcl_subscription_t setpoint_subscriber;    // Receives motor angular velocity setpoint

// Timers
rcl_timer_t timer;                         // Timer to execute control loop at intervals

// Messages
std_msgs__msg__Float32 setpoint_msg;       // Setpoint message
std_msgs__msg__Float32 motor_output_msg;   // Motor angular velocity message
std_msgs__msg__Float32 error_msg;          // Error message
std_msgs__msg__Float32 motor_input_msg;    // Control signal message
std_msgs__msg__Int32 encoder_count_msg;    // Encoder count message

// ===== MOTOR CONTROL VARIABLES =====
// Encoder tracking
volatile int32_t encoder_count = 0;
static int32_t last_encoder_count = 0;
volatile uint32_t last_encoder_time = 0;

// Angular velocity
float angular_velocity = 0.0;
float previous_angular_velocity = 0.0;
float filtered_angular_velocity = 0.0;

// PID control parameters
float kp = 20.0;                   // Proportional gain
float ki = 30.0;                   // Integral gain
float kd = 0.5;                    // Derivative gain

// PID variables
const float sampling_time = 1.0 / CONTROL_LOOP_FREQ_HZ;
float setpoint = 0.0;             
float error = 0.0, error_prev1 = 0.0, error_prev2 = 0.0;
float output = 0.0, previous_output = 0.0, applied_output = 0.0;        

// Filter coefficients
const float filter_a_1 = 0.854, filter_b_0 = 0.0728, filter_b_1 = filter_b_0;

// Debug variables
volatile bool encoder_activity = false;  // Flag to indicate encoder activity

// ===== FUNCTION PROTOTYPES =====
// ROS 2 entity management
bool create_entities();
void destroy_entities();

// Interrupt Service Routines
void IRAM_ATTR encoderA_ISR();
void IRAM_ATTR encoderB_ISR();

// Callbacks
void setpoint_subscription_callback(const void *msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// Motor control
float set_motor_output(float output_value);

// ===== ROS 2 ENTITY MANAGEMENT FUNCTIONS =====
// ROS 2 entity creation function
bool create_entities() {
  // Initializes memory allocation for micro-ROS operations
  allocator = rcl_get_default_allocator();

  // Creates a ROS 2 support structure to manage the execution context
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&motor_node, "motor_controller", "", &support));

  // Create subscriber for setpoint - RELIABLE QoS
  RCCHECK(rclc_subscription_init_default(
    &setpoint_subscriber,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "setpoint"));

  // Create publisher for motor output (actual velocity) - BEST EFFORT QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &motor_output_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output"));

  // Create publisher for error - BEST EFFORT QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &error_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "error"));

  // Create publisher for motor input (control signal) - RELIABLE QoS
  RCCHECK(rclc_publisher_init_default(
    &motor_input_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input"));

  // Create publisher for encoder count - BEST EFFORT QoS
  RCCHECK(rclc_publisher_init_best_effort(
    &encoder_count_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_count"));

  // Create timer that triggers control loop at fixed interval
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(CONTROL_LOOP_PERIOD_MS),
    timer_callback));

  // Initialize executor with enough handles for all entities
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Add subscription and timer to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &setpoint_subscriber, &setpoint_msg, &setpoint_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  return true;
}

// Cleanup function for ROS 2 entities
void destroy_entities() {
  // Set timeout to zero for immediate cleanup
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Finalize all ROS 2 entities
  rcl_subscription_fini(&setpoint_subscriber, &motor_node);
  rcl_publisher_fini(&motor_output_publisher, &motor_node);
  rcl_publisher_fini(&error_publisher, &motor_node);
  rcl_publisher_fini(&motor_input_publisher, &motor_node);
  rcl_publisher_fini(&encoder_count_publisher, &motor_node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&motor_node);
  rclc_support_fini(&support);
}

// ===== INTERRUPT SERVICE ROUTINES =====
void IRAM_ATTR encoderA_ISR() {
  encoder_activity = true;
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  encoder_count += (A == B) ? 1 : -1;  // Clockwise or counter-clockwise
  last_encoder_time = micros();
}

void IRAM_ATTR encoderB_ISR() {
  encoder_activity = true;
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);
  encoder_count += (A != B) ? 1 : -1;  // Clockwise or counter-clockwise
  last_encoder_time = micros(); 
}

// ===== CALLBACK FUNCTIONS =====
// Callback function for the setpoint subscriber
void setpoint_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  setpoint = msg->data;  
}

// Timer callback function
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);  // Prevents compiler warnings about unused parameter

  if (timer != NULL) {
    // Calculate elapsed time for encoder timeout detection
    uint32_t now = micros();
    
    // Check if encoder has timed out (no pulses recently)
    if (now - last_encoder_time > ENCODER_TIMEOUT_US) {
      // No recent encoder activity, assume motor is stopped
      angular_velocity = 0.0;
    } else {
      // Get current encoder count (atomic read to prevent race condition)
      noInterrupts();
      int32_t current_encoder_count = encoder_count;
      interrupts();
      
      // Calculate angular velocity (rad/s) based on encoder count change
      int32_t delta_count = current_encoder_count - last_encoder_count;
      last_encoder_count = current_encoder_count;
      angular_velocity = 2.0 * M_PI * (float)delta_count / (ENCODER_RESOLUTION * ENCODER_GEAR_RATIO * sampling_time);
    }
    
    // Apply low-pass filter to smooth velocity
    filtered_angular_velocity = filter_a_1 * filtered_angular_velocity + 
                                filter_b_0 * angular_velocity + 
                                filter_b_1 * previous_angular_velocity;
    previous_angular_velocity = angular_velocity;

    // Compute the control signal using PID controller
    error_prev2 = error_prev1;
    error_prev1 = error;
    error = setpoint - filtered_angular_velocity;
    
    previous_output = output;
    output = previous_output + 
             (kp + (kd / sampling_time)) * error +
             (-kp - 2.0 * (kd / sampling_time) + (ki * sampling_time)) * error_prev1 +
             (kd / sampling_time) * error_prev2;
    
    // Apply control signal to motor and get actual applied value
    applied_output = set_motor_output(output);
    
    // Debug indicator for encoder activity (only update if state changes)
    static bool last_led_state = LOW;
    if (encoder_activity != last_led_state) {
      digitalWrite(LED_DEBUG_PIN, encoder_activity ? HIGH : LOW);
      last_led_state = encoder_activity;
    }
    encoder_activity = false; 

    // Publish all relevant data
    motor_output_msg.data = filtered_angular_velocity;
    RCSOFTCHECK(rcl_publish(&motor_output_publisher, &motor_output_msg, NULL));
    
    error_msg.data = error;
    RCSOFTCHECK(rcl_publish(&error_publisher, &error_msg, NULL));
    
    motor_input_msg.data = applied_output;
    RCSOFTCHECK(rcl_publish(&motor_input_publisher, &motor_input_msg, NULL));
    
    encoder_count_msg.data = encoder_count;
    RCSOFTCHECK(rcl_publish(&encoder_count_publisher, &encoder_count_msg, NULL));
  }
}

// ===== MOTOR CONTROL FUNCTIONS =====

// Function to set motor output
float set_motor_output(float output_value) {
  float limited_output = output_value;
  
  // Limit the output to the PWM bounds.
  if(limited_output < -PWM_MAX) limited_output = -PWM_MAX;
  if(limited_output > PWM_MAX) limited_output = PWM_MAX;
  
  // Set motor direction based on the sign of the output.
  if(limited_output < 0) {
    digitalWrite(PWM_IN1, LOW);
    digitalWrite(PWM_IN2, HIGH);
  } else if(limited_output > 0) {
    digitalWrite(PWM_IN1, HIGH);
    digitalWrite(PWM_IN2, LOW);
  } else {
    digitalWrite(PWM_IN1, LOW);
    digitalWrite(PWM_IN2, LOW); 
  }

  // Set PWM duty cycle based on the absolute value of output.
  ledcWrite(PWM_CHNL, (int)fabs(limited_output));
  
  return limited_output;
}

// ===== ARDUINO SETUP FUNCTION =====
void setup() {
  // Initializes communication between ESP32 and the ROS 2 Agent (serial)
  set_microros_transports();
  
  // Initialize state machine
  state = WAITING_AGENT;

  // Setup microcontroller pins
  pinMode(LED_DEBUG_PIN, OUTPUT);
  digitalWrite(LED_DEBUG_PIN, LOW);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_IN1, OUTPUT);
  pinMode(PWM_IN2, OUTPUT);

  // Setup PWM for motor control
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  // Attach interrupts to both encoder pins for quadrature detection
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_ISR, CHANGE);

  // Initialize motor to stopped state
  set_motor_output(0);

  // Initialize timing variable
  last_encoder_time = micros();
}

// ===== ARDUINO LOOP FUNCTION =====
void loop() {
  switch (state) {
    case WAITING_AGENT:
      // Periodically check if ROS 2 agent is available
      EXECUTE_EVERY_N_MS(AGENT_CHECK_INTERVAL_MS, 
        state = (RMW_RET_OK == rmw_uros_ping_agent(PING_TIMEOUT_MS, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      // Try to create ROS 2 entities
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      // Check if agent is still connected
      EXECUTE_EVERY_N_MS(PING_CHECK_INTERVAL_MS, 
        state = (RMW_RET_OK == rmw_uros_ping_agent(PING_TIMEOUT_MS, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        // Process ROS 2 communications
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_TIME_MS)));
      }
      break;

    case AGENT_DISCONNECTED:
      // Clean up if agent disconnects
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}