#include <ESP32Encoder.h>

#define MOTOR_PWM_FWD 25
#define MOTOR_PWM_REV 26
#define ENCODER_A 18
#define ENCODER_B 19

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_FWD_CHANNEL 0
#define PWM_REV_CHANNEL 1

#define PID_FREQ_HZ 100
#define PID_PERIOD_MS (1000 / PID_FREQ_HZ)

#define MOTOR_KP 2.0
#define MOTOR_KI 0.5
#define MOTOR_KD 0.05
#define INTEGRAL_LIMIT 50.0

#define ENCODER_CPR 48
#define WHEEL_RADIUS 0.035

ESP32Encoder encoder;
hw_timer_t *pid_timer = NULL;

volatile bool pid_tick = false;

float velocity_setpoint = 0.0;
float pid_integral = 0.0;
int32_t prev_count = 0;

void IRAM_ATTR onPidTimer() {
    pid_tick = true;
}

void setup() {
    Serial.begin(115200);
    
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachFullQuad(ENCODER_A, ENCODER_B);
    encoder.clearCount();
    
    ledcSetup(PWM_FWD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_REV_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_FWD, PWM_FWD_CHANNEL);
    ledcAttachPin(MOTOR_PWM_REV, PWM_REV_CHANNEL);
    
    ledcWrite(PWM_FWD_CHANNEL, 0);
    ledcWrite(PWM_REV_CHANNEL, 0);
    
    pid_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(pid_timer, &onPidTimer, true);
    timerAlarmWrite(pid_timer, PID_PERIOD_MS * 1000, true);
    timerAlarmEnable(pid_timer);
    
    Serial.println("Motor PID Test");
    Serial.println("Commands: setpoint <velocity>");
    Serial.println("Example: setpoint 0.5");
    Serial.println("time,velocity_cmd,velocity_actual,pwm");
}

void loop() {
    handle_serial();
    
    if (pid_tick) {
        pid_tick = false;
        pid_loop();
    }
}

void handle_serial() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("setpoint ")) {
            velocity_setpoint = cmd.substring(9).toFloat();
            Serial.print("Setpoint: ");
            Serial.println(velocity_setpoint);
        }
    }
}

void pid_loop() {
    float dt = PID_PERIOD_MS / 1000.0;
    
    int32_t current_count = encoder.getCount();
    int32_t delta = current_count - prev_count;
    prev_count = current_count;
    
    float revolutions = (float)delta / (float)ENCODER_CPR;
    float distance = revolutions * 2.0 * 3.14159265 * WHEEL_RADIUS;
    float velocity_actual = distance / dt;
    
    float error = velocity_setpoint - velocity_actual;
    
    float P = MOTOR_KP * error;
    
    pid_integral += error;
    if (pid_integral > INTEGRAL_LIMIT) pid_integral = INTEGRAL_LIMIT;
    if (pid_integral < -INTEGRAL_LIMIT) pid_integral = -INTEGRAL_LIMIT;
    float I = MOTOR_KI * pid_integral;
    
    float D = 0.0;
    
    float pwm = P + I + D;
    
    set_motor_pwm(pwm);
    
    Serial.print(millis() / 1000.0, 3);
    Serial.print(",");
    Serial.print(velocity_setpoint, 3);
    Serial.print(",");
    Serial.print(velocity_actual, 3);
    Serial.print(",");
    Serial.println(pwm, 1);
}

void set_motor_pwm(float pwm) {
    int pwm_value = (int)(pwm);
    if (pwm_value > 255) pwm_value = 255;
    if (pwm_value < -255) pwm_value = -255;
    
    if (pwm_value > 0) {
        ledcWrite(PWM_FWD_CHANNEL, pwm_value);
        ledcWrite(PWM_REV_CHANNEL, 0);
    } else if (pwm_value < 0) {
        ledcWrite(PWM_FWD_CHANNEL, 0);
        ledcWrite(PWM_REV_CHANNEL, -pwm_value);
    } else {
        ledcWrite(PWM_FWD_CHANNEL, 0);
        ledcWrite(PWM_REV_CHANNEL, 0);
    }
}