#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>

// ── Pin Definitions ─────────────────────────────────────────
#define LEFT_SENSOR   34
#define CENTER_SENSOR 35
#define RIGHT_SENSOR  32

#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENA 25
#define ENB 33

#define LED_PIN 2

// ── Motor PWM config ─────────────────────────────────────────
const int PWM_FREQ      = 1000;
const int PWM_RES_BITS  = 8;        // 0–255

// ── Drive constants ──────────────────────────────────────────
const int MAX_PWM       = 250;      // maximum motor PWM (out of 255)
const int MIN_PWM       = 60;      // minimum PWM to overcome stiction
const int STEER_MIX     = 160;       // differential authority


// ── Safety timeout ───────────────────────────────────────────
unsigned long last_cmd_time = 0;
const int TIMEOUT_MS = 300;

// ── micro-ROS handles ────────────────────────────────────────
rcl_publisher_t    ir_publisher;
rcl_subscription_t cmd_subscriber;

std_msgs__msg__Int32             ir_msg;
std_msgs__msg__Float32MultiArray cmd_msg;

float joy_data_buf[2];   // [0]=steering, [1]=throttle

rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;

unsigned long last_pub_time = 0;

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ── Motor helpers ────────────────────────────────────────────

// Low-level driver — accepts signed PWM per side (-255 to +255).
// Enforces MIN_PWM so motors don't stall on tiny commands.
void setMotor(int right, int left) {
  right = constrain(right, -MAX_PWM, MAX_PWM);
  left  = constrain(left,  -MAX_PWM, MAX_PWM);

  // Below stiction threshold → round up to MIN_PWM (or zero)
  if (right > 0  && right < MIN_PWM) right = MIN_PWM;
  if (right < 0  && right > -MIN_PWM) right = -MIN_PWM;
  if (left  > 0  && left  < MIN_PWM) left  = MIN_PWM;
  if (left  < 0  && left  > -MIN_PWM) left  = -MIN_PWM;

  // RIGHT motor — IN1/IN2 + ENA
  if (right >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    right = -right;
  }

  // LEFT motor — IN3/IN4 + ENB
  if (left >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    left = -left;
  }

  ledcWrite(ENA, right);
  ledcWrite(ENB, left);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// ── /cmd_joy subscriber callback ─────────────────────────────
// Receives Float32MultiArray [steering, throttle] from Python PID node.
//   steering : -1.0 (full left)  …  +1.0 (full right)
//   throttle : -1.0 (full back)  …  +1.0 (full forward)

void cmd_callback(const void* msgin) {
  const std_msgs__msg__Float32MultiArray* msg =
      (const std_msgs__msg__Float32MultiArray*)msgin;

  if (msg->data.size < 2) return;

  float x = msg->data.data[0];   // steering
  float y = msg->data.data[1];   // throttle

  last_cmd_time = millis();

  // Small dead-zone — removes jitter around zero
  if (fabsf(x) < 0.05f) x = 0.0f;
  if (fabsf(y) < 0.05f) y = 0.0f;

  // Differential drive mix
  //   throttle component  : both motors equal
  //   steering component  : one side faster, other slower
  int throttle_pwm = (int)(y * MAX_PWM);
  int steer_pwm    = (int)(x * STEER_MIX);

  int rightMotor = constrain(throttle_pwm + steer_pwm, -MAX_PWM, MAX_PWM);
  int leftMotor  = constrain(throttle_pwm - steer_pwm, -MAX_PWM, MAX_PWM);

  setMotor(rightMotor, leftMotor);

  Serial.printf("[CMD] x:%.3f y:%.3f  R:%d L:%d\n",
                x, y, rightMotor, leftMotor);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Sensor pins
  pinMode(LEFT_SENSOR,   INPUT);
  pinMode(CENTER_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR,  INPUT);

  // Motor / LED pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // PWM channels
  ledcAttach(ENA, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_RES_BITS);
  stopMotors();

  // Wire Float32MultiArray message buffer before executor runs
  cmd_msg.data.data     = joy_data_buf;
  cmd_msg.data.size     = 2;
  cmd_msg.data.capacity = 2;

  // Connect to micro-ROS agent (blinks LED while waiting)
  while (true) {
    set_microros_wifi_transports("S23ultra", "Samip2062", "10.172.163.70", 8888);
    if (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) break;
    digitalWrite(LED_PIN, HIGH); delay(250);
    digitalWrite(LED_PIN, LOW);  delay(250);
  }
  digitalWrite(LED_PIN, HIGH);   // solid = connected

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "line_follower_node", "", &support));

  // Publisher: IR sensor readings → /sensor_data/ir_sensors
  RCCHECK(rclc_publisher_init_best_effort(
    &ir_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sensor_data/ir_sensors"));

  // Subscriber: drive commands ← /cmd_joy
  RCCHECK(rclc_subscription_init_best_effort(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/cmd_joy"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_subscriber, &cmd_msg, &cmd_callback, ON_NEW_DATA));

  ir_msg.data   = 0;
  last_cmd_time = millis();

  Serial.println("Line Follower Ready");
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  // Process any incoming /cmd_joy messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  // Safety watchdog — stop if no command for TIMEOUT_MS
  if (millis() - last_cmd_time > TIMEOUT_MS) {
    stopMotors();
  }

  // Publish IR readings at ~100 Hz (every 10 ms)
  if (micros() - last_pub_time >= 10000) {
    last_pub_time = micros();

    int left   = digitalRead(LEFT_SENSOR);
    int center = digitalRead(CENTER_SENSOR);
    int right  = digitalRead(RIGHT_SENSOR);

    // Pack L/C/R into single Int32 — e.g. L=1 C=0 R=1 → 101
    ir_msg.data = (left * 100) + (center * 10) + right;

    RCSOFTCHECK(rcl_publish(&ir_publisher, &ir_msg, NULL));
    Serial.printf("[PUB] L:%d C:%d R:%d → %d\n",
                  left, center, right, ir_msg.data);
  }
}
