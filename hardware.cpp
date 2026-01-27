#include "hardware.h"
#include "config.h"
#include <ESP32Encoder.h>

ESP32Encoder encoder_cart;
ESP32Encoder encoder_pendulum;
ESP32Encoder encoder_motor1;
ESP32Encoder encoder_motor2;
ESP32Encoder encoder_motor3;
ESP32Encoder encoder_motor4;

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_MOTOR1_FWD_CHANNEL = 0;
const int PWM_MOTOR1_REV_CHANNEL = 1;
const int PWM_MOTOR2_FWD_CHANNEL = 2;
const int PWM_MOTOR2_REV_CHANNEL = 3;
const int PWM_MOTOR3_FWD_CHANNEL = 4;
const int PWM_MOTOR3_REV_CHANNEL = 5;
const int PWM_MOTOR4_FWD_CHANNEL = 6;
const int PWM_MOTOR4_REV_CHANNEL = 7;

static bool button_mode_last = false;
static bool button_action_last = false;
static bool button_calibrate_last = false;

void hardware_init() {
    pinMode(BUTTON_MODE_PIN, INPUT_PULLUP);
    pinMode(BUTTON_ACTION_PIN, INPUT_PULLUP);
    pinMode(BUTTON_CALIBRATE_PIN, INPUT_PULLUP);
    
    ESP32Encoder::useInternalWeakPullResistors = UP;
    
    encoder_cart.attachFullQuad(ENCODER_CART_A, ENCODER_CART_B);
    encoder_pendulum.attachFullQuad(ENCODER_PENDULUM_A, ENCODER_PENDULUM_B);
    encoder_motor1.attachFullQuad(ENCODER_MOTOR1_A, ENCODER_MOTOR1_B);
    encoder_motor2.attachFullQuad(ENCODER_MOTOR2_A, ENCODER_MOTOR2_B);
    encoder_motor3.attachFullQuad(ENCODER_MOTOR3_A, ENCODER_MOTOR3_B);
    encoder_motor4.attachFullQuad(ENCODER_MOTOR4_A, ENCODER_MOTOR4_B);
    
    encoder_cart.clearCount();
    encoder_pendulum.clearCount();
    encoder_motor1.clearCount();
    encoder_motor2.clearCount();
    encoder_motor3.clearCount();
    encoder_motor4.clearCount();
    
    ledcSetup(PWM_MOTOR1_FWD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR1_REV_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR2_FWD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR2_REV_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR3_FWD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR3_REV_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR4_FWD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_MOTOR4_REV_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    
    ledcAttachPin(MOTOR1_PWM_FWD, PWM_MOTOR1_FWD_CHANNEL);
    ledcAttachPin(MOTOR1_PWM_REV, PWM_MOTOR1_REV_CHANNEL);
    ledcAttachPin(MOTOR2_PWM_FWD, PWM_MOTOR2_FWD_CHANNEL);
    ledcAttachPin(MOTOR2_PWM_REV, PWM_MOTOR2_REV_CHANNEL);
    ledcAttachPin(MOTOR3_PWM_FWD, PWM_MOTOR3_FWD_CHANNEL);
    ledcAttachPin(MOTOR3_PWM_REV, PWM_MOTOR3_REV_CHANNEL);
    ledcAttachPin(MOTOR4_PWM_FWD, PWM_MOTOR4_FWD_CHANNEL);
    ledcAttachPin(MOTOR4_PWM_REV, PWM_MOTOR4_REV_CHANNEL);
    
    ledcWrite(PWM_MOTOR1_FWD_CHANNEL, 0);
    ledcWrite(PWM_MOTOR1_REV_CHANNEL, 0);
    ledcWrite(PWM_MOTOR2_FWD_CHANNEL, 0);
    ledcWrite(PWM_MOTOR2_REV_CHANNEL, 0);
    ledcWrite(PWM_MOTOR3_FWD_CHANNEL, 0);
    ledcWrite(PWM_MOTOR3_REV_CHANNEL, 0);
    ledcWrite(PWM_MOTOR4_FWD_CHANNEL, 0);
    ledcWrite(PWM_MOTOR4_REV_CHANNEL, 0);
}

bool hardware_read_button_mode() {
    bool current = (digitalRead(BUTTON_MODE_PIN) == LOW);
    bool rising_edge = current && !button_mode_last;
    button_mode_last = current;
    return rising_edge;
}

bool hardware_read_button_action() {
    bool current = (digitalRead(BUTTON_ACTION_PIN) == LOW);
    bool rising_edge = current && !button_action_last;
    button_action_last = current;
    return rising_edge;
}

bool hardware_read_button_calibrate() {
    bool current = (digitalRead(BUTTON_CALIBRATE_PIN) == LOW);
    bool rising_edge = current && !button_calibrate_last;
    button_calibrate_last = current;
    return rising_edge;
}

int32_t hardware_read_encoder_cart() {
    return (int32_t)encoder_cart.getCount();
}

int32_t hardware_read_encoder_pendulum() {
    return (int32_t)encoder_pendulum.getCount();
}

int32_t hardware_read_encoder_motor1() {
    return (int32_t)encoder_motor1.getCount();
}

int32_t hardware_read_encoder_motor2() {
    return (int32_t)encoder_motor2.getCount();
}

int32_t hardware_read_encoder_motor3() {
    return (int32_t)encoder_motor3.getCount();
}

int32_t hardware_read_encoder_motor4() {
    return (int32_t)encoder_motor4.getCount();
}

void hardware_zero_encoders() {
    encoder_cart.clearCount();
    encoder_pendulum.clearCount();
}

static void set_motor_pwm(int fwd_channel, int rev_channel, float pwm) {
    int pwm_value = (int)(pwm);
    if (pwm_value > 255) pwm_value = 255;
    if (pwm_value < -255) pwm_value = -255;
    
    if (pwm_value > 0) {
        ledcWrite(fwd_channel, pwm_value);
        ledcWrite(rev_channel, 0);
    } else if (pwm_value < 0) {
        ledcWrite(fwd_channel, 0);
        ledcWrite(rev_channel, -pwm_value);
    } else {
        ledcWrite(fwd_channel, 0);
        ledcWrite(rev_channel, 0);
    }
}

void hardware_set_motor1_pwm(float pwm) {
    set_motor_pwm(PWM_MOTOR1_FWD_CHANNEL, PWM_MOTOR1_REV_CHANNEL, pwm);
}

void hardware_set_motor2_pwm(float pwm) {
    set_motor_pwm(PWM_MOTOR2_FWD_CHANNEL, PWM_MOTOR2_REV_CHANNEL, pwm);
}

void hardware_set_motor3_pwm(float pwm) {
    set_motor_pwm(PWM_MOTOR3_FWD_CHANNEL, PWM_MOTOR3_REV_CHANNEL, pwm);
}

void hardware_set_motor4_pwm(float pwm) {
    set_motor_pwm(PWM_MOTOR4_FWD_CHANNEL, PWM_MOTOR4_REV_CHANNEL, pwm);
}

unsigned long hardware_millis() {
    return millis();
}