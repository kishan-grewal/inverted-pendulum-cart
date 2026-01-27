#include "config.h"
#include "control.h"
#include "observer.h"
#include "motor.h"
#include "hardware.h"

enum SystemState {
    STATE_STARTUP,
    STATE_CALIBRATING,
    STATE_IDLE,
    STATE_BALANCE,
    STATE_SPRINT,
    STATE_SWING_TEST
};

enum SystemMode {
    MODE_BALANCE,
    MODE_SPRINT
};

SystemState current_state = STATE_STARTUP;
SystemMode selected_mode = MODE_BALANCE;

hw_timer_t *control_timer = NULL;
hw_timer_t *motor_timer = NULL;

volatile bool control_tick = false;
volatile bool motor_tick = false;

float state[4] = {0.0, 0.0, 0.0, 0.0};
float x_dot_cmd = 0.0;
unsigned long time_ms = 0;
unsigned long swing_test_start = 0;

void IRAM_ATTR onControlTimer() {
    control_tick = true;
}

void IRAM_ATTR onMotorTimer() {
    motor_tick = true;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    hardware_init();
    control_init();
    observer_init();
    motor_init();
    
    control_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(control_timer, &onControlTimer, true);
    timerAlarmWrite(control_timer, CONTROL_PERIOD_MS * 1000, true);
    timerAlarmEnable(control_timer);
    
    motor_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(motor_timer, &onMotorTimer, true);
    timerAlarmWrite(motor_timer, MOTOR_PID_PERIOD_MS * 1000, true);
    timerAlarmEnable(motor_timer);
    
    current_state = STATE_CALIBRATING;
    
    Serial.println("time,x,x_dot,theta,theta_dot,x_dot_cmd,state");
}

void loop() {
    handle_buttons();
    handle_serial_commands();
    
    if (control_tick) {
        control_tick = false;
        control_loop();
    }
    
    if (motor_tick) {
        motor_tick = false;
        motor_loop();
    }
}

void handle_buttons() {
    if (hardware_read_button_mode()) {
        if (current_state == STATE_IDLE) {
            selected_mode = (selected_mode == MODE_BALANCE) ? MODE_SPRINT : MODE_BALANCE;
            Serial.print("Selected mode: ");
            Serial.println(selected_mode == MODE_BALANCE ? "BALANCE" : "SPRINT");
        }
    }
    
    if (hardware_read_button_action()) {
        if (current_state == STATE_IDLE) {
            if (selected_mode == MODE_BALANCE) {
                current_state = STATE_BALANCE;
                observer_reset();
                control_reset();
                motor_reset();
                Serial.println("Starting BALANCE mode");
            } else if (selected_mode == MODE_SPRINT) {
                current_state = STATE_SPRINT;
                observer_reset();
                control_reset();
                motor_reset();
                Serial.println("Starting SPRINT mode");
            }
        } else if (current_state == STATE_BALANCE || current_state == STATE_SPRINT || current_state == STATE_SWING_TEST) {
            current_state = STATE_IDLE;
            motor_set_velocity_command(0.0);
            Serial.println("Stopped, returning to IDLE");
        }
    }
    
    if (hardware_read_button_calibrate()) {
        current_state = STATE_CALIBRATING;
        motor_set_velocity_command(0.0);
        Serial.println("Calibrating");
    }
}

void handle_serial_commands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "lqr") {
            control_set_controller(CTRL_LQR);
            Serial.println("Controller: LQR");
        } else if (cmd == "pid") {
            control_set_controller(CTRL_PID);
            Serial.println("Controller: PID");
        } else if (cmd == "pole") {
            control_set_controller(CTRL_POLE);
            Serial.println("Controller: Pole Placement");
        } else if (cmd == "swing") {
            if (current_state == STATE_IDLE) {
                current_state = STATE_SWING_TEST;
                swing_test_start = hardware_millis();
                Serial.println("Starting swing test");
            }
        } else if (cmd == "cal") {
            current_state = STATE_CALIBRATING;
            motor_set_velocity_command(0.0);
            Serial.println("Calibrating");
        }
    }
}

void control_loop() {
    time_ms = hardware_millis();
    float dt = CONTROL_PERIOD_MS / 1000.0;
    
    if (current_state == STATE_CALIBRATING) {
        hardware_zero_encoders();
        observer_reset();
        control_reset();
        motor_reset();
        state[0] = 0.0;
        state[1] = 0.0;
        state[2] = 0.0;
        state[3] = 0.0;
        x_dot_cmd = 0.0;
        motor_set_velocity_command(0.0);
        current_state = STATE_IDLE;
        Serial.println("Calibration complete");
        return;
    }
    
    int32_t cart_counts = hardware_read_encoder_cart();
    int32_t pend_counts = hardware_read_encoder_pendulum();
    
    float x_measured = (float)cart_counts / (float)ENCODER_CPR_CART;
    float theta_measured = (float)pend_counts / (float)ENCODER_CPR_PEND * 2.0 * 3.14159265;
    
    float y_measured[2] = {x_measured, theta_measured};
    observer_update(y_measured, x_dot_cmd, dt, state);
    
    if (current_state == STATE_BALANCE || current_state == STATE_SPRINT) {
        x_dot_cmd = control_compute(state, dt);
        motor_set_velocity_command(x_dot_cmd);
    } else if (current_state == STATE_SWING_TEST) {
        x_dot_cmd = 0.0;
        motor_set_velocity_command(0.0);
        
        if ((hardware_millis() - swing_test_start) > 20000) {
            current_state = STATE_IDLE;
            Serial.println("Swing test complete");
        }
    } else {
        x_dot_cmd = 0.0;
        motor_set_velocity_command(0.0);
    }
    
    log_data();
}

void motor_loop() {
    float dt = MOTOR_PID_PERIOD_MS / 1000.0;
    motor_update(dt);
}

void log_data() {
    Serial.print(time_ms / 1000.0, 3);
    Serial.print(",");
    Serial.print(state[0], 6);
    Serial.print(",");
    Serial.print(state[1], 6);
    Serial.print(",");
    Serial.print(state[2], 6);
    Serial.print(",");
    Serial.print(state[3], 6);
    Serial.print(",");
    Serial.print(x_dot_cmd, 6);
    Serial.print(",");
    
    if (current_state == STATE_IDLE) {
        Serial.println("IDLE");
    } else if (current_state == STATE_BALANCE) {
        Serial.println("BALANCE");
    } else if (current_state == STATE_SPRINT) {
        Serial.println("SPRINT");
    } else if (current_state == STATE_SWING_TEST) {
        Serial.println("SWING");
    } else if (current_state == STATE_CALIBRATING) {
        Serial.println("CALIBRATING");
    } else {
        Serial.println("UNKNOWN");
    }
}