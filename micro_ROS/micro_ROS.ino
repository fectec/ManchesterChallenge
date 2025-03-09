// ===== LIBRARY INCLUDES =====
#include <micro_ros_arduino.h>          // micro-ROS for ESP32
#include <rcl/rcl.h>                    // Core ROS 2 Client Library
#include <rcl/error_handling.h>         // Error handling utilities
#include <rclc/rclc.h>                  // Micro-ROS Client Library
#include <rclc/executor.h>              // Micro-ROS Executor
#include <std_msgs/msg/int32.h>         // Message type for encoder count
#include <std_msgs/msg/float32.h>       // Message type for float values
#include <rmw_microros/rmw_microros.h>  // For QoS settings
#include <stdio.h>                      // Standard I/O library
#include <math.h>                       // Standard math library

// ===== TIMING CONFIGURATION =====
// All timing values in milliseconds (ms)
#define CONTROL_LOOP_FREQ_HZ     100    // Control loop frequency in Hz (100Hz = 10ms period)
#define EXECUTOR_SPIN_TIME_MS    1      // Time for executor to process messages (ms)
#define AGENT_CHECK_INTERVAL_MS  500    // How often to check for agent in WAITING_AGENT state
#define PING_CHECK_INTERVAL_MS   200    // How often to ping agent in CONNECTED state
#define PING_TIMEOUT_MS          100    // How long to wait for ping response

// Calculate control loop period from frequency
#define CONTROL_LOOP_PERIOD_MS   (1000 / CONTROL_LOOP_FREQ_HZ)

// ===== HARDWARE DEFINITIONS =====
// Encoder pins and configuration
#define ENCODER_A 35              // Encoder channel A pin
#define ENCODER_B 34              // Encoder channel B pin
#define ENCODER_RESOLUTION 1024   // Number of ticks per full revolution

// H-Bridge (Motor Driver) configuration
#define PWM_PIN   4               // PWM output
#define PWM_IN1   18              // Motor direction pin 1
#define PWM_IN2   15              // Motor direction pin 2
#define PWM_FRQ   980             // PWM frequency
#define PWM_RES   8               // PWM resolution (8 bits)
#define PWM_CHNL  0               // PWM channel

// ===== HELPER MACROS =====
// Executes a function and returns false if it fails
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes function but ignores failures
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
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

// ===== MOTOR CONTROL VARIABLES =====
// Encoder tracking
volatile int32_t encoder_count = 0;
volatile int32_t previous_encoder_count = 0;  // To store the previous encoder count
unsigned long previous_time = 0;              // To store the previous time
float angular_velocity = 0.0;                 // Current motor velocity

// PID control parameters
float kp = 1.0;                   // Proportional gain
float ki = 0.0;                   // Integral gain
float kd = 0.0;                   // Derivative gain

// PID state variables
float setpoint_value = 0.0;       // Setpoint received from the ROS topic
float dt = CONTROL_LOOP_PERIOD_MS / 1000.0; // Control loop period in seconds
float prev_error = 0.0;           // Previous error for derivative calculation
float integral = 0.0;             // Integral sum of errors
float control_signal = 0.0;       // Motor control signal

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
rcl_publisher_t motor_output_publisher;    // Publishes actual motor velocity
rcl_publisher_t error_publisher;           // Publishes error (setpoint - actual)
rcl_publisher_t motor_input_publisher;     // Publishes control signal to motor
rcl_publisher_t encoder_pulses_publisher;  // Publishes raw encoder count

// Subscribers
rcl_subscription_t setpoint_subscriber;    // Receives velocity setpoint

// Timers
rcl_timer_t timer;                         // Timer to execute control loop at intervals

// Messages
std_msgs__msg__Float32 setpoint_msg;       // Setpoint message
std_msgs__msg__Float32 motor_output_msg;   // Motor velocity message
std_msgs__msg__Float32 error_msg;          // Error message
std_msgs__msg__Float32 motor_input_msg;    // Control signal message
std_msgs__msg__Int32 encoder_pulses_msg;   // Encoder count message

// ===== FUNCTION PROTOTYPES =====
// ROS 2 entity management
bool create_entities();
void destroy_entities();

// Encoder interrupt service routines
void IRAM_ATTR encoder_isr_A();
void IRAM_ATTR encoder_isr_B();

// Control system functions
float get_angular_velocity();
float compute_PID(float angular_velocity);
void apply_control_signal(float control_signal);

// Callback functions
void setpoint_subscription_callback(const void *msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// ===== INTERRUPT SERVICE ROUTINES =====
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

// ===== CALLBACK FUNCTIONS =====
// Callback function for the setpoint subscriber
void setpoint_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  setpoint_value = msg->data;  
}

// Timer callback function
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);  // Prevents compiler warnings about unused parameter

  if (timer != NULL) {
    // 1. Get the current angular velocity from encoder
    angular_velocity = get_angular_velocity();
    
    // 2. Compute PID control signal
    control_signal = compute_PID(angular_velocity);
    
    // 3. Apply control signal to motor
    apply_control_signal(control_signal);
    
    // 4. Publish all relevant data
    
    // Publish current angular velocity (motor output)
    motor_output_msg.data = angular_velocity;
    RCSOFTCHECK(rcl_publish(&motor_output_publisher, &motor_output_msg, NULL));
    
    // Publish error
    error_msg.data = setpoint_value - angular_velocity;
    RCSOFTCHECK(rcl_publish(&error_publisher, &error_msg, NULL));
    
    // Publish control signal (motor input)
    motor_input_msg.data = control_signal;
    RCSOFTCHECK(rcl_publish(&motor_input_publisher, &motor_input_msg, NULL));
    
    // Publish raw encoder pulses
    encoder_pulses_msg.data = encoder_count;
    RCSOFTCHECK(rcl_publish(&encoder_pulses_publisher, &encoder_pulses_msg, NULL));
  }
}

// ===== MOTOR CONTROL FUNCTIONS =====
// Function to convert encoder count to angular velocity
float get_angular_velocity() {
  // Get the current time
  unsigned long current_time = millis();
  
  // Calculate the time difference (in seconds)
  float time_diff = (current_time - previous_time) / 1000.0;  // Convert from milliseconds to seconds
  
  // Get the change in encoder count
  int32_t encoder_diff = encoder_count - previous_encoder_count;
  
  // Convert encoder count to angle (in radians)
  float angle_diff = encoder_diff * (2 * M_PI / ENCODER_RESOLUTION);
  
  // Calculate the angular velocity in radians per second
  float angular_velocity = angle_diff / time_diff;
  
  // Store the current encoder count and time for the next update
  previous_encoder_count = encoder_count;
  previous_time = current_time;
  
  // Return the angular velocity in radians per second
  return angular_velocity;
}

// Function to calculate motor input using PID
float compute_PID(float angular_velocity) {
  // Calculate error (difference between setpoint and actual velocity)
  float error_value = setpoint_value - angular_velocity;
  
  // Calculate integral term and apply anti-windup
  integral += error_value * dt;
  integral = constrain(integral, -1.0, 1.0);
  
  // Calculate derivative term
  float derivative = (error_value - prev_error) / dt;
  
  // Calculate final control signal
  float control_signal = (kp * error_value) + (ki * integral) + (kd * derivative);
  
  // Store current error for next iteration
  prev_error = error_value;
  
  // Constrain control signal to valid range (-1.0 to 1.0)
  return constrain(control_signal, -1.0, 1.0);
}

// Function to apply control signal to motor
void apply_control_signal(float control_signal) {
  // Convert -1 to 1 range into a valid PWM duty cycle (0-255)
  int pwm_value = (int)(fabs(control_signal) * ((1 << PWM_RES) - 1));
  
  // Motor control logic - set direction based on sign
  if (control_signal > 0) {
    digitalWrite(PWM_IN1, HIGH);
    digitalWrite(PWM_IN2, LOW);
  } else if (control_signal < 0) {
    digitalWrite(PWM_IN1, LOW);
    digitalWrite(PWM_IN2, HIGH);
  } else {
    digitalWrite(PWM_IN1, LOW);
    digitalWrite(PWM_IN2, LOW);
  }

  // Apply PWM value to motor
  ledcWrite(PWM_CHNL, pwm_value);
}

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
  // Rationale: Motor commands are critical and must always be received
  RCCHECK(rclc_subscription_init_default(
    &setpoint_subscriber,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "setpoint"));

  // Create publisher for motor output (actual velocity) - BEST EFFORT QoS
  // Rationale: High-frequency sensor-like data where latest values are more important
  RCCHECK(rclc_publisher_init_best_effort(
    &motor_output_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output"));

  // Create publisher for error - BEST EFFORT QoS
  // Rationale: Diagnostic data where latest values matter more than delivery guarantee
  RCCHECK(rclc_publisher_init_best_effort(
    &error_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "error"));

  // Create publisher for motor input (control signal) - RELIABLE QoS
  // Rationale: Important for debugging and monitoring the control system
  RCCHECK(rclc_publisher_init_default(
    &motor_input_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input"));

  // Create publisher for encoder pulses - BEST EFFORT QoS
  // Rationale: High-frequency sensor data where occasional losses are acceptable
  RCCHECK(rclc_publisher_init_best_effort(
    &encoder_pulses_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_pulses"));

  // Create timer that triggers control loop at fixed interval
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(CONTROL_LOOP_PERIOD_MS),
    timer_callback));

  // Initialize executor with enough handles for all entities
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

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
  rcl_publisher_fini(&encoder_pulses_publisher, &motor_node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&motor_node);
  rclc_support_fini(&support);
}

// ===== ARDUINO SETUP FUNCTION =====
void setup() {
  // Initializes communication between ESP32 and the ROS 2 Agent (serial)
  set_microros_transports();
  
  // Initialize state machine
  state = WAITING_AGENT;

  // Setup microcontroller pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_IN1, OUTPUT);
  pinMode(PWM_IN2, OUTPUT);

  // Attach interrupts to encoder pins A & B for quadrature detection
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isr_B, CHANGE);

  // Setup PWM
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);
  
  // Initialize timing variables
  previous_time = millis();
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
      };
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