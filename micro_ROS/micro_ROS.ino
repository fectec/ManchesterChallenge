// Include libraries to be used

#include <micro_ros_arduino.h>          // micro-ROS for ESP32
#include <rcl/rcl.h>                    // Core ROS 2 Client Library
#include <rcl/error_handling.h>         // Error handling utilities
#include <rclc/rclc.h>                  // Micro-ROS Client Library
#include <rclc/executor.h>              // Micro-ROS Executor
#include <std_msgs/msg/int32.h>         // Message type for encoder count
#include <std_msgs/msg/float32.h>       // Message type
#include <stdio.h>                      // Standard I/O library
#include <math.h>                       // Standard math library

// micro-ROS entity declarations

// Nodes
rcl_node_t motor_node;                  // ROS 2 Node running on the MCU

// Instantiate executor and its support classes
rclc_executor_t executor;               // Manages task execution (timers, callbacks, etc.)
rclc_support_t support;                 // Handles initialization & communication setup
rcl_allocator_t allocator;              // Manages memory allocation

// Subscribers
rcl_subscription_t setpoint_subscriber;

// Publishers
rcl_publisher_t motor_output_publisher;
rcl_publisher_t error_publisher;
rcl_publisher_t motor_input_publisher;
rcl_publisher_t encoder_pulses_publisher;

// Timers
rcl_timer_t timer;                      // Timer to execute functions at intervals

// Messages
std_msgs__msg__Float32 setpoint;
std_msgs__msg__Float32 motor_output;
std_msgs__msg__Float32 error;
std_msgs__msg__Float32 motor_input;
std_msgs__msg__Int32 encoder_pulses;

// micro-ROS connection state machine and function prototypes

enum states {
  WAITING_AGENT,        
  AGENT_AVAILABLE,      
  AGENT_CONNECTED,      
  AGENT_DISCONNECTED    
} state;

bool create_entities();
void destroy_entities();

// Macro definitions

// Executes a function and returns false if it fails
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes function but ignores failures
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X) do { 
  static volatile int64_t init = -1; 
  if (init == -1) {init = uxr_millis();} 
  if (uxr_millis() - init > MS) {X; init = uxr_millis();} 
} while(0)

// Hardware definitions

// Encoder
#define ENCODER_A 35
#define ENCODER_B 34 

// H-Bridge
#define PWM_PIN   4             // PWM output
#define PWM_IN1   18            // Motor direction pin 1
#define PWM_IN2   15            // Motor direction pin 2
#define PWM_FRQ   980           // PWM frequency
#define PWM_RES   8             // PWM resolution (8 bits)
#define PWM_CHNL  0             // PWM channel

// Interrupt Service Routines (ISRs)

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

// Callback function for the setpoint subscriber
void setpoint_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  setpoint_value = msg->data;  // Store the received setpoint value
}

// Timer callback function
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);                                           // Prevents compiler warnings about an unused parameter

  if (timer != NULL) {                                                   // Ensures the timer event is valid before executing actions

  }
}

// PID control variables, constants and function prototypes
#define ENCODER_RESOLUTION 1024   // Number of ticks per full revolution

volatile int32_t encoder_count = 0;
volatile int32_t previous_encoder_count = 0;  // To store the previous encoder count
unsigned long previous_time = 0;              // To store the previous time

float Kp = 1.0;                   // Proportional gain
float Ki = 0.0;                   // Integral gain
float Kd = 0.0;                   // Derivative gain

float prev_error = 0.0;           // Previous error for derivative calculation
float integral = 0.0;             // Integral sum of errors
float setpoint_value = 0.0;       // Setpoint received from the ROS topic
float dt = 0.1;                   // 100 ms timer

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

// Function to calculate motor input
float compute_PID(float angular_velocity) {
  float error = setpoint_value - angular_velocity;
  integral += error * dt;
  integral = constrain(integral, -1.0, 1.0);
  float derivative = (error - prev_error) / dt;
  float motor_input = (kp * error) + (ki * integral) + (kd * derivative);
  prev_error = error;
  return constrain(motor_input, -1.0, 1.0);
}

// Function to apply control signal
void apply_control_signal(float motor_input) {
  // Convert -1 to 1 range into a valid PWM duty cycle (0-255)
  int pwm_value = (int)(fabs(motor_input) * ((1 << PWM_RES) - 1));
  
  // Motor control logic
  if (motor_input > 0) {
    digitalWrite(PWM_IN1, HIGH);
    digitalWrite(PWM_IN2, LOW);
  } else if (motor_input < 0) {
    digitalWrite(PWM_IN1, LOW);
    digitalWrite(PWM_IN2, HIGH);
  } else {
    digitalWrite(PWM_IN1, LOW);
    digitalWrite(PWM_IN2, LOW);
  }

  ledcWrite(PWM_CHNL, pwm_value);
}

void setup() {
  // Initializes communication between ESP32 and the ROS 2 Agent (serial)
  set_microros_transports();

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
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}

// ROS 2 entity creation and cleanup functions

bool create_entities() {
  // Initializes memory allocation for micro-ROS operations
  allocator = rcl_get_default_allocator();

  // Creates a ROS 2 support structure to manage the execution context
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&motor_node, "motor", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &setpoint_subscriber,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "setpoint"));

  // Create publishers
  RCCHECK(rclc_publisher_init_decmd_pwmfault(
    &motor_output_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output"));

  RCCHECK(rclc_publisher_init_default(
    &error_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "error"));

  RCCHECK(rclc_publisher_init_default(
    &motor_input_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input"));

  RCCHECK(rclc_publisher_init_default(
    &encoder_pulses_publisher,
    &motor_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_pulses"));

  // Create timer
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initializes the micro-ROS executor (zero initialized to avoid memory problems)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  // Register subscriptor & timer
  RCCHECK(rclc_executor_add_subscription(&executor, &setpoint_subscriber, &setpoint, &setpoint_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  return true
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

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