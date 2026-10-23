#include <util/atomic.h>
#include <base_controller.h>
#include <naive_state_estimator.h>
#include <voltage_reader.h>
#include <base_protocol.h>
#include <banana_checksum.h>
#include <banana_scheduler.h>
#include "hardware_defs.h"

// Helper functions:
void left_encoder_interrupt(void);
void right_encoder_interrupt(void);

// Task functions:
static void update_wheel_encoder(void);
static void read_sensor_states(void);
static void update_motor_output(void);
static void update_led_status(void);
static void serial_sync_data(void);
static void serial_recv_command_data(void);
static void predict_state_estimator(void);

// Global data:
typedef struct {
    int16_t lost_packet_counter;
    int32_t recv_sequence_id;
    int16_t send_sequence_id;
    uint8_t warn_iter;
    uint16_t battery_adc_value;
    float   battery_voltage;
    int32_t led_timer;
    uint8_t led_state;
    int16_t led_off_period;
    uint8_t debug_code;
} vehicle_data_t;

static vehicle_data_t           VehicleData;
static base_controller_t        BaseController;
static schedule_table_t         S_Table;
static naive_state_estimator_t  NSE;
static int16_t                  __LeftEncoder;
static int16_t                  __RightEncoder;

// Wheel encoders interrupt handles:
void left_encoder_interrupt(void) {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        __LeftEncoder++;
    }
}
void right_encoder_interrupt(void) {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        __RightEncoder++;
    }
}

void setup(void) {
    // Global variables:
    VehicleData.lost_packet_counter = 0;
    VehicleData.recv_sequence_id    = 0;
    VehicleData.send_sequence_id    = 0;
    VehicleData.warn_iter           = 0;
    VehicleData.led_state           = 0;
    VehicleData.led_timer           = 0;
    VehicleData.led_off_period      = -1;
    __LeftEncoder                   = 0;
    __RightEncoder                  = 0;

    // Encoder Interrupts:
    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_PIN),
        left_encoder_interrupt,
        RISING
    );
    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_PIN),
        right_encoder_interrupt,
        RISING
    );

    // Setup IO pins:
    pinMode(LEFT_DIR_PIN,       OUTPUT);
    pinMode(RIGHT_DIR_PIN,      OUTPUT);
    pinMode(LEFT_DIR_INV_PIN,   OUTPUT);
    pinMode(RIGHT_DIR_INV_PIN,  OUTPUT);
    pinMode(LEFT_PWM_PIN,       OUTPUT);
    pinMode(RIGHT_PWM_PIN,      OUTPUT);
    pinMode(LED_PIN,            OUTPUT);
    pinMode(STBY_PIN,           OUTPUT);

    // Write IO pins initial values:
    digitalWrite(LEFT_DIR_PIN,      HIGH);
    digitalWrite(RIGHT_DIR_PIN,     HIGH);
    digitalWrite(LEFT_DIR_INV_PIN,  LOW);
    digitalWrite(RIGHT_DIR_INV_PIN, LOW);
    digitalWrite(STBY_PIN,          HIGH);
    analogWrite(LED_PIN,            0);

    // Initialize sub-modules:
    base_controller_init(
        &BaseController,
        MOTOR_KP,
        MOTOR_KI,
        MOTOR_KD,
        RAMP_LINEAR_ACCEL,
        MOTOR_MAX_PWM,
        WHEEL_DIAMETER,
        WHEEL_TRACK,
        ENCODER_RESOLUTION
    );
    /*base_controller_set_ramp_on(&BaseController);*/
    base_controller_set_ramp_off(&BaseController);
    nse_init(&NSE, NSE_GPS_W, NSE_ODOM_W, NSE_COMPASS_W, NSE_PRED_W);
    scheduler_init(&S_Table, MAIN_LOOP_PERIOD, MAIN_LOOP_MAX, 10);
    VehicleData.led_timer = scheduler_get_timestamp(&S_Table);

    // ROS Serial init:
    Serial.begin(ROS_SERIAL_RATE);
}

void loop(void) {
    // Use a (manually coded) scheduling table to balance CPU load:
    // loop rate 10Hz, each iter%10 run at 1Hz
    // can control task range 1-10Hz with step size 1Hz
    //
    // Task Frequency Requirements:
    // Freq(Hz)     Task
    // 10           serial_sync_data
    // 10           update_motor_output
    // 10           update_wheel_encoder
    // 5            predict_state_estimator
    // 2            update_led_status
    // 1            read_sensor_states
    if(scheduler_loop_enter(&S_Table, millis())) {
        // (10Hz) Universal tasks for all iterations:
        serial_sync_data();
        update_wheel_encoder();
        update_motor_output();

        // Seperated tasks:
        switch(scheduler_get_iter(&S_Table)) {
            case 0:
                predict_state_estimator();
                break;
            case 1:
                update_led_status();
                break;
            case 2:
                predict_state_estimator();
                break;
            case 3:
                read_sensor_states();
                break;
            case 4:
                predict_state_estimator();
                break;
            case 5:
                break;
            case 6:
                predict_state_estimator();
                break;
            case 7:
                update_led_status();
                break;
            case 8:
                predict_state_estimator();
                break;
            case 9:
                break;
            default:
                break;
        }
        scheduler_loop_exit(&S_Table);
    }
}

static void update_wheel_encoder(void) {
    int16_t curr_left_encoder, curr_right_encoder;

    // Compute wheel encoder velocity:
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        curr_left_encoder = __LeftEncoder;
        __LeftEncoder = 0;
    }
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        curr_right_encoder = __RightEncoder;
        __RightEncoder = 0;
    }

    base_controller_update(
        &BaseController,
        curr_left_encoder,
        curr_right_encoder,
        scheduler_get_timestamp(&S_Table)
    );

    nse_update_wheel_odom(
        &NSE,
        (double) base_controller_get_linear_vel(&BaseController),
        (double) base_controller_get_angular_vel(&BaseController)
    );
}

static void read_sensor_states(void) {
    VehicleData.battery_adc_value = analogRead(BATTERY_PIN);
    VehicleData.battery_voltage = get_voltage_value(
        VehicleData.battery_adc_value,
        REF_VOLTAGE,
        ADC_R1,
        ADC_R2
    );
}

static void update_motor_output(void) {
    // Update motor direction signal:
    if(base_controller_get_left_motor_forward(&BaseController)) {
        // Left motor forward:
        digitalWrite(LEFT_DIR_PIN, LOW);
        digitalWrite(LEFT_DIR_INV_PIN, HIGH);
    } else {
        // Left motor backward:
        digitalWrite(LEFT_DIR_PIN, HIGH);
        digitalWrite(LEFT_DIR_INV_PIN, LOW);
    }
    if(base_controller_get_right_motor_forward(&BaseController)) {
        // Right motor forward:
        digitalWrite(RIGHT_DIR_PIN, LOW);
        digitalWrite(RIGHT_DIR_INV_PIN, HIGH);
    } else {
        // Right motor backward:
        digitalWrite(RIGHT_DIR_PIN, HIGH);
        digitalWrite(RIGHT_DIR_INV_PIN, LOW);
    }

    VehicleData.debug_code = BaseController.right_motor.indicator.mode;

    analogWrite(
        LEFT_PWM_PIN,
        base_controller_get_left_motor_pwm(
            &BaseController
        )
    );
    analogWrite(
        RIGHT_PWM_PIN,
        base_controller_get_right_motor_pwm(
            &BaseController
        )
    );
}

static void update_led_status(void) {
    if(VehicleData.battery_voltage < 6.0) {
        // Battery critical
        VehicleData.led_off_period = 200;
    } else if(VehicleData.battery_voltage < 6.5) {
        // Battery low
        VehicleData.led_off_period = 500;
    } else {
        // Normal state
        VehicleData.led_off_period = 2000;
    }

    if(VehicleData.led_off_period == -1) {
        // LED keeps off
        analogWrite(LED_PIN, 0);
    } else {
        if(VehicleData.led_state == 0) {
            // LED off state
            if(
                scheduler_get_timestamp(&S_Table) - VehicleData.led_timer > \
                    VehicleData.led_off_period
            ) {
                // LED off -> on
                VehicleData.led_state = 1;
                VehicleData.led_timer = scheduler_get_timestamp(&S_Table);
                analogWrite(LED_PIN, LED_PWM_VALUE);
            }
        } else {
            // LED on state
            if(
                scheduler_get_timestamp(&S_Table) - VehicleData.led_timer > \
                    LED_ON_PERIOD
            ) {
                // LED on -> off
                VehicleData.led_state = 0;
                VehicleData.led_timer = scheduler_get_timestamp(&S_Table);
                analogWrite(LED_PIN, 0);
            }
        }
    }

    // LED timer increment:
    /*if(VehicleData.led_timer + 50 < 32767) {*/
        /*VehicleData.led_timer += 50;*/
    /*}*/
}

static void serial_sync_data(void) {
    ros_status_frame_t send_frame;

    send_frame.header               = FRAMEID_ROS_STATUS;
    send_frame.sequence_id          = VehicleData.send_sequence_id++;
    send_frame.indicator.left_dir   = base_controller_get_left_motor_forward(&BaseController);
    send_frame.indicator.right_dir  = base_controller_get_right_motor_forward(&BaseController);
    send_frame.x                    = NSE.x;
    send_frame.y                    = NSE.y;
    send_frame.v                    = NSE.v;
    send_frame.yaw                  = NSE.yaw;
    send_frame.vyaw                 = NSE.vyaw;
    send_frame.v_left               = BaseController.left_motor.v_encoder;
    send_frame.v_right              = BaseController.right_motor.v_encoder;
    send_frame.left_pwm             = base_controller_get_left_motor_pwm(&BaseController);
    send_frame.right_pwm            = base_controller_get_right_motor_pwm(&BaseController);
    send_frame.left_err             = BaseController.left_motor.pid.err_prev;
    send_frame.right_err            = BaseController.right_motor.pid.err_prev;
    send_frame.battery_adc_value    = VehicleData.battery_adc_value;
    send_frame.debug_code           = VehicleData.debug_code;
    send_frame.checksum             = compute_checksum((char *) &send_frame, sizeof(ros_status_frame_t));

    Serial.write((char *) &send_frame, sizeof(ros_status_frame_t));

    // Wait ACK
    delay(5);
    serial_recv_command_data();
}

static void serial_recv_command_data(void) {
    ros_command_frame_t recv_frame;

    if(Serial.available() > 0) {
        if(Serial.readBytes(
            (char *) &recv_frame,
            sizeof(ros_command_frame_t)
        ) != sizeof(ros_command_frame_t)) {
            // Packet size mismatch, skip
            return;
        }

        if(compute_checksum(
            (char *) &recv_frame,
            sizeof(ros_command_frame_t)
        ) != recv_frame.checksum) {
            // Checksum mismatch, skip
            return;
        }

        if(recv_frame.header != FRAMEID_ROS_COMMAND) {
            // Header mismatch, skip
            return;
        }

        // Match sequence id to check lost packets:
        if((VehicleData.recv_sequence_id + 1) != recv_frame.sequence_id) {
            VehicleData.lost_packet_counter++;
        }
        VehicleData.recv_sequence_id = recv_frame.sequence_id;

        // Unpack data:
        base_controller_set_cmd(
            &BaseController,
            recv_frame.cmd_x,
            recv_frame.cmd_yaw
        );

        // Process toggles:
        if(recv_frame.toggle.reset_odom) {
            nse_init(&NSE, NSE_GPS_W, NSE_ODOM_W, NSE_COMPASS_W, NSE_PRED_W);
        }
        if(recv_frame.toggle.ramp_on) {
            base_controller_set_ramp_on(&BaseController);
        } else if(recv_frame.toggle.ramp_off) {
            base_controller_set_ramp_off(&BaseController);
        }
        if(recv_frame.toggle.motor_on) {
            digitalWrite(STBY_PIN, HIGH);
        } else if(recv_frame.toggle.motor_off) {
            digitalWrite(STBY_PIN, LOW);
        }
    }
}

static void predict_state_estimator(void) {
    nse_predict(&NSE, scheduler_get_timestamp(&S_Table));
}
