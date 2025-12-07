#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>

#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP32Servo.h>

// ================= WiFi =================
#define WIFI_SSID "Home"
#define WIFI_PASS "13781374"
#define AGENT_IP  "192.168.1.101"
#define AGENT_PORT 8888

// ================= Pins =================
#define LED_PIN 2

#define PWMA  17
#define AIN1  18
#define AIN2  5
#define STBY  19
#define BIN1  21
#define BIN2  22
#define PWMB  23

#define PIN_BUZZER 15
#define PIN_SERVO  26

#define TRIG_FRONT 27
#define ECHO_FRONT 14
#define TRIG_REAR  25
#define ECHO_REAR  34

#define SDA_PIN 4
#define SCL_PIN 16

// ============== Objects ================
MPU6050 mpu(Wire);
Servo front_servo;

// ============== ROS2 ====================
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

rcl_publisher_t pub_imu;
rcl_publisher_t pub_range_front;
rcl_publisher_t pub_range_rear;

rcl_subscription_t sub_motor_pwm;
rcl_subscription_t sub_servo;
rcl_subscription_t sub_buzzer;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range range_front_msg;
sensor_msgs__msg__Range range_rear_msg;

std_msgs__msg__Float32MultiArray motor_msg;
std_msgs__msg__Int32 servo_cmd_msg;
std_msgs__msg__Bool buzzer_msg;

// ============== Helpers =================
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 30000);
  return (d == 0) ? 999.0f : d * 0.0343f / 2.0f;
}

void motorInit() {
  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);
  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void setMotorLeft(float s) {
  s = constrain(s, -1, 1);
  int pwm = (int)(fabs(s) * 255);
  if (s > 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else if (s < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); pwm = 0; }
  ledcWrite(0, pwm);
}

void setMotorRight(float s) {
  s = constrain(s, -1, 1);
  int pwm = (int)(fabs(s) * 255);
  if (s > 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
  else if (s < 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); pwm = 0; }
  ledcWrite(1, pwm);
}

// ============== Callbacks ================
void sub_motor_pwm_cb(const void *msgin) {
  auto *msg = (std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 2) {
    setMotorLeft(msg->data.data[0]);
    setMotorRight(msg->data.data[1]);
  }
}

void sub_servo_cb(const void *msgin) {
  auto *msg = (std_msgs__msg__Int32 *)msgin;
  front_servo.write(constrain(msg->data, 0, 180));
}

void sub_buzzer_cb(const void *msgin) {
  auto *msg = (std_msgs__msg__Bool *)msgin;
  digitalWrite(PIN_BUZZER, msg->data);
}

// ============== Setup ====================
unsigned long last_sensor = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // خط ۱ – داخل setup() بعد از WiFi.begin اینو بذار (حذف کن while بی‌نهایت رو)
  WiFi.begin(WIFI_SSID, WIFI_PASS, 0, NULL, true);

  unsigned long wifi_timeout = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_timeout < 20000) {
    delay(100);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed!");
    error_loop();  // اگر وای‌فای وصل نشد، LED چشمک بزنه
  }
  Serial.println("WiFi connected: " + WiFi.localIP().toString());
  digitalWrite(PIN_BUZZER, HIGH);
  delay(200);
  digitalWrite(PIN_BUZZER, LOW);
  delay(200);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);

  if (rmw_uros_ping_agent(1000, 3) == RMW_RET_OK) {  // timeout 1000 ms
    Serial.println("Agent reachable!");

    // دو بار بوق بعد از اتصال Agent
    for (int i = 0; i < 2; i++) {
      digitalWrite(PIN_BUZZER, HIGH);
      delay(150);
      digitalWrite(PIN_BUZZER, LOW);
      delay(150);
    }
  } else {
    Serial.println("Agent not reachable!");
  }


  motorInit();
  front_servo.attach(PIN_SERVO, 1000, 2000);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_REAR,  OUTPUT); pinMode(ECHO_REAR, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  mpu.calcOffsets();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot", "", &support);

  rclc_publisher_init_best_effort(
      &pub_imu, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/robot/imu");

  rclc_publisher_init_best_effort(
      &pub_range_front, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "/robot/range_front");

  rclc_publisher_init_best_effort(
      &pub_range_rear, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "/robot/range_rear");

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.depth = 10;

  rclc_subscription_init(
      &sub_motor_pwm, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/robot/motor_pwm", &qos);

  rclc_subscription_init(
      &sub_servo, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/robot/servo", &qos);

  rclc_subscription_init(
      &sub_buzzer, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/robot/buzzer", &qos);

  motor_msg.data.data = (float *)malloc(sizeof(float) * 2);
  motor_msg.data.size = 2;
  motor_msg.data.capacity = 2;

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &sub_motor_pwm, &motor_msg, &sub_motor_pwm_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_servo, &servo_cmd_msg, &sub_servo_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_buzzer, &buzzer_msg, &sub_buzzer_cb, ON_NEW_DATA);

  range_front_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_front_msg.field_of_view = 0.26;
  range_front_msg.min_range = 0.02;
  range_front_msg.max_range = 4.0;
  range_front_msg.header.frame_id.data = (char *)"front";
  range_front_msg.header.frame_id.size = 5;
  range_front_msg.header.frame_id.capacity = 10;

  range_rear_msg = range_front_msg;
  range_rear_msg.header.frame_id.data = (char *)"rear";
  range_rear_msg.header.frame_id.size = 4;
}

// ============== Loop ====================
void loop() {
  rclc_executor_spin_some(&executor, 5000);

  unsigned long now = millis();
  if (now - last_sensor >= 50) {
    last_sensor = now;

    mpu.update();
    imu_msg.header.stamp.sec = now / 1000;
    imu_msg.header.stamp.nanosec = (now % 1000) * 1000000UL;

    imu_msg.linear_acceleration.x = mpu.getAccX();
    imu_msg.linear_acceleration.y = mpu.getAccY();
    imu_msg.linear_acceleration.z = mpu.getAccZ();

    imu_msg.angular_velocity.x = mpu.getGyroX() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = mpu.getGyroY() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = mpu.getGyroZ() * DEG_TO_RAD;

    rcl_publish(&pub_imu, &imu_msg, NULL);

    range_front_msg.range = getDistance(TRIG_FRONT, ECHO_FRONT);
    range_front_msg.header.stamp = imu_msg.header.stamp;
    rcl_publish(&pub_range_front, &range_front_msg, NULL);

    range_rear_msg.range = getDistance(TRIG_REAR, ECHO_REAR);
    range_rear_msg.header.stamp = imu_msg.header.stamp;
    rcl_publish(&pub_range_rear, &range_rear_msg, NULL);
  }
}
