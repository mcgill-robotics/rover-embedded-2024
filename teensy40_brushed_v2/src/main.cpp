#include <Arduino.h>
#include "hardware_pins.h"
#include "driver_motor.h"
#include "model_encoder.h"
#include "model_sensor.h"
#include "motor.h"

#include <memory>

// CONFIGURATION
#define BRUSHED_BOARD_TEST 1

#define OUTA_Pin ENCPIN1_1
#define OUTB_Pin ENCPIN1_2

// DUMB POINTERS
driver_motor mot1;
driver_motor mot2;
driver_motor mot3;

// TESTER VARIABLES
int stage = 0;
int pos = 90;
int tolerance = 0.5;

// FUNCTION DECLARATIONS
void brushed_board_tester();
void brushed_board_loop();
void lim1ISR();
void lim2ISR();
void lim3ISR();
void lim4ISR();
void lim5ISR();
void lim6ISR();

// Define the size of the moving average window
const uint32_t MOVING_AVERAGE_SIZE = 10;
float cur1_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
float cur2_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
float cur3_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
int cur1_voltage_buffer_idx = 0;
int cur2_voltage_buffer_idx = 0;
int cur3_voltage_buffer_idx = 0;
float moving_average(float new_reading, float *buffer, int buffer_size, int *buffer_idx)
{
    float sum = 0;
    float oldest_reading = buffer[*buffer_idx];

    // Add the new reading to the buffer.
    buffer[*buffer_idx] = new_reading;
    *buffer_idx = (*buffer_idx + 1) % buffer_size; // Circular buffer

    // Sum all readings.
    for (int i = 0; i < buffer_size; i++)
    {
        sum += buffer[i];
    }
    sum -= oldest_reading;

    return sum / buffer_size;
}

// Ensure counter is initialized
int counter = 0;

Rotation2d *current_rotation;

// LIMIT SWITCHES
IntervalTimer lim_watchdog_timer;
const unsigned long DEBOUNCE_DELAY = 50;

volatile bool lim1_state = false;
volatile bool lim2_state = false;
volatile bool lim3_state = false;
volatile bool lim4_state = false;
volatile bool lim5_state = false;
volatile bool lim6_state = false;

void setup()
{
    SerialUSB.begin(115200);

    std::unique_ptr<model_encoder> enc1 = std::make_unique<model_encoder>();
    std::unique_ptr<model_sensor> cur1 = std::make_unique<model_sensor>();

    std::unique_ptr<model_encoder> enc2 = std::make_unique<model_encoder>();
    std::unique_ptr<model_sensor> cur2 = std::make_unique<model_sensor>();

    std::unique_ptr<model_encoder> enc3 = std::make_unique<model_encoder>();
    std::unique_ptr<model_sensor> cur3 = std::make_unique<model_sensor>();

    // Initialize encoders
    // 43000 clicks for wrist pitch, TODO check others
    // Only using 1 & 2 becase 1 & 3 conflicts
    enc1->initialize_encoder(0, 0, 32580, 1);       //new small servo estimate for resolution
    enc2->initialize_encoder(0, 0, 43000, 2);       
    // enc3->initialize_encoder(0, 0, 43000, 3);

    // Initialize current sensors
    cur1->initialize_sensor(CURRENT_SENSE_A);
    cur3->initialize_sensor(CURRENT_SENSE_C);
    cur2->initialize_sensor(CURRENT_SENSE_B);

    // Initialize motors
    mot1.attach_encoder(std::move(enc1));
    mot2.attach_encoder(std::move(enc2));
    mot3.attach_encoder(std::move(enc3));

    mot1.attach_current_sensor(std::move(cur1));
    mot2.attach_current_sensor(std::move(cur2));
    mot3.attach_current_sensor(std::move(cur3));

    mot1.initialize_motor(0, PWMPIN1, DIRPIN1, nSLEEP1, 5.0, 0.0);
    mot2.initialize_motor(0, PWMPIN2, DIRPIN2, nSLEEP2, 5.0, 0.0);
    mot3.initialize_motor(0, PWMPIN3, DIRPIN3, nSLEEP3, 5.0, 0.0);

    // Initialize Pins
    // pinMode(PWMPIN1, OUTPUT);
    // pinMode(DIRPIN1, OUTPUT);
    // pinMode(nSLEEP1, OUTPUT);
    // pinMode(PWMPIN2, OUTPUT);
    // pinMode(DIRPIN2, OUTPUT);
    // pinMode(nSLEEP2, OUTPUT);
    // pinMode(PWMPIN3, OUTPUT);
    // pinMode(DIRPIN3, OUTPUT);
    // pinMode(nSLEEP3, OUTPUT);

    // pinMode(ENCPIN3_1, INPUT);
    // pinMode(ENCPIN3_2, INPUT);

    // Limit Switches
    pinMode(LIM_1, INPUT);
    pinMode(LIM_2, INPUT);
    pinMode(LIM_3, INPUT);
    pinMode(LIM_4, INPUT);
    pinMode(LIM_5, INPUT);
    pinMode(LIM_6, INPUT);
    attachInterrupt(digitalPinToInterrupt(LIM_1), lim1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_2), lim2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_3), lim3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_4), lim4ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_5), lim5ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_6), lim6ISR, CHANGE);

    // Watchdog timer to prevent bouncing or stuck limit switches
    lim_watchdog_timer.begin([]()
                             {
        lim1_state = digitalRead(LIM_1);
        lim2_state = digitalRead(LIM_2);
        lim3_state = digitalRead(LIM_3);
        lim4_state = digitalRead(LIM_4);
        lim5_state = digitalRead(LIM_5);
        lim6_state = digitalRead(LIM_6); },
                             DEBOUNCE_DELAY * 1000);

    // Initialize Motor
    digitalWrite(nSLEEP1, HIGH);
    digitalWrite(DIRPIN1, HIGH);
    digitalWrite(nSLEEP2, HIGH);
    digitalWrite(DIRPIN2, HIGH);
    digitalWrite(nSLEEP3, HIGH);
    digitalWrite(DIRPIN3, HIGH);

    analogWrite(PWMPIN1, 40);
    analogWrite(PWMPIN2, 40);
    analogWrite(PWMPIN3, 40);

    // Modelling
    current_rotation = Rotation2d::getRotationFromDeg(0);
    SerialUSB.println("Done Setup");
}

void loop()
{
#if BRUSHED_BOARD_TEST == 1
    brushed_board_tester();
#else
    brushed_board_loop();
#endif
}

void brushed_board_loop()
{
    delay(100);

    if (stage)
    {
        SerialUSB.println("Already reached target position");
    }
    else
    {
        mot1.set_target_position(pos);
        mot1.closed_loop_control_tick();
        float enc1_angle = mot1._encoder->get_angle();
        SerialUSB.printf("enc1_angle: %8.4f \n", enc1_angle);

        if (enc1_angle <= pos + tolerance && enc1_angle >= pos - tolerance && stage == 0)
        {
            SerialUSB.println("Reached target position");
            //pos = -60;
            //stage++;
        }
    }
}

void lim1ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY)
    {
        SerialUSB.println("LIM_1 triggered");
        last_interrupt_time = interrupt_time;
        lim1_state = digitalRead(LIM_1);
    }
}

void lim2ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY)
    {
        SerialUSB.println("LIM_2 triggered");
        last_interrupt_time = interrupt_time;
        lim2_state = digitalRead(LIM_2);
    }
}

void lim3ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY)
    {
        SerialUSB.println("LIM_3 triggered");
        last_interrupt_time = interrupt_time;
        lim3_state = digitalRead(LIM_3);
    }
}

void lim4ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY)
    {
        SerialUSB.println("LIM_4 triggered");
        last_interrupt_time = interrupt_time;
        lim4_state = digitalRead(LIM_4);
    }
}

void lim5ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY)
    {
        SerialUSB.println("LIM_5 triggered");
        last_interrupt_time = interrupt_time;
        lim5_state = digitalRead(LIM_5);
    }
}

void lim6ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY)
    {
        SerialUSB.println("LIM_6 triggered");
        last_interrupt_time = interrupt_time;
        lim6_state = digitalRead(LIM_6);
    }
}

void brushed_board_tester()
{
    delay(100);

    // TEST LIMIT SWITCHES --------------------------------------------------------------------
    // printf("LIM_1: %d, LIM_2: %d, LIM_3: %d, LIM_4: %d, LIM_5: %d, LIM_6: %d\n",
    //        lim1_state, lim2_state, lim3_state, lim4_state, lim5_state, lim6_state);

    // TEST CURRENT SENSOR --------------------------------------------------------------------
    mot1._current_sensor->read_sensor_value();
    mot2._current_sensor->read_sensor_value();
    mot3._current_sensor->read_sensor_value();

    // TODO check implementation of get_current()
    float cur1_current = mot1._current_sensor->get_current();
    float cur2_current = mot2._current_sensor->get_current();
    float cur3_current = mot3._current_sensor->get_current();

    // Middle value is 1.5V
    float cur1_voltage = mot1._current_sensor->get_raw_voltage() - 1.5;
    float cur2_voltage = mot2._current_sensor->get_raw_voltage() - 1.5;
    float cur3_voltage = mot3._current_sensor->get_raw_voltage() - 1.5;
    float smoothed_cur1_voltage = moving_average(cur1_voltage, cur1_voltage_buffer, MOVING_AVERAGE_SIZE, &cur1_voltage_buffer_idx);
    float smoothed_cur2_voltage = moving_average(cur2_voltage, cur2_voltage_buffer, MOVING_AVERAGE_SIZE, &cur2_voltage_buffer_idx);
    float smoothed_cur3_voltage = moving_average(cur3_voltage, cur3_voltage_buffer, MOVING_AVERAGE_SIZE, &cur3_voltage_buffer_idx);

    // SerialUSB.printf("cur1_current: %8.4f, cur2_current: %8.4f, cur3_current: %8.4f ",
    //                  cur1_current, cur2_current, cur3_current);
    SerialUSB.printf("cur1_voltage: %8.4f, cur2_voltage: %8.4f, cur3_voltage: %8.4f ",
                     smoothed_cur1_voltage, smoothed_cur2_voltage, smoothed_cur3_voltage);

    // TEST ENCODER --------------------------------------------------------------------
    // Channel 3 is conflicted, unused
    mot1._encoder->poll_encoder_angle();
    mot2._encoder->poll_encoder_angle();
    // mot3._encoder->poll_encoder_angle();
    float enc1_angle = mot1._encoder->get_angle();
    float rel_angle = mot1._encoder->get_relative_angle();
    float enc1_tick = mot1._encoder->_encoder->read();

    // float enc2_angle = mot2._encoder->get_angle();
    // float enc3_angle = mot3._encoder->get_angle();
    SerialUSB.printf("enc1_angle: %8.4f, rel_angle: %8.4f, tick: %8.4f",
                     enc1_angle, rel_angle, enc1_tick);
    
    if(enc1_angle <= 2881 && enc1_angle >= 2879)
    {
        SerialUSB.printf("\n2880 here^");
        analogWrite(PWMPIN1, 0);
        while(true);
    }

    // TEST GRAVITY COMPENSATION --------------------------------------------------------------------
    // double output;
    // current_rotation->setAngleRad(current_angle / 360 * 2 * PI);
    // maintainStateProto(*current_rotation, &output);
    // printf("current output: %8.4f\r\n", output);
    // printf("Current voltage: %8.4f\n", output);
    // if (output < 0)
    // {
    //     digitalWrite(DIR1, HIGH);
    // }
    // else
    // {
    //     digitalWrite(DIR1, LOW);
    // }
    // int analog_write_output = (output / 24) * 255;
    // printf("Current analog output: %d\n", analog_write_output);
    // analogWrite(PWM1, analog_write_output);

    // ENCODER PINS TEST --------------------------------------------------------------------
    // SerialUSB.printf("ENCPIN1_1: %d, ENCPIN1_2: %d ", digitalRead(ENCPIN1_1), digitalRead(ENCPIN1_2));
    // SerialUSB.printf("ENCPIN2_1: %d, ENCPIN2_2: %d ", digitalRead(ENCPIN2_1), digitalRead(ENCPIN2_2));
    // SerialUSB.printf("ENCPIN3_1: %d, ENCPIN3_2: %d\n", digitalRead(ENCPIN3_1), digitalRead(ENCPIN3_2));
    // if (digitalRead(OUTA_Pin) == LOW) // If OUTA is LOW
    // {
    //     if (digitalRead(OUTB_Pin) == LOW) // If OUTB is also LOW... CCK
    //     {
    //         while (digitalRead(OUTB_Pin) == LOW)
    //         {
    //         }; // wait for OUTB to go HIGH
    //         counter--;
    //         while (digitalRead(OUTA_Pin) == LOW)
    //         {
    //         };         // wait for OUTA to go HIGH
    //         delay(10); // wait for some more time
    //     }
    //     else if (digitalRead(OUTB_Pin) == HIGH) // If OUTB is HIGH
    //     {
    //         while (digitalRead(OUTB_Pin) == HIGH)
    //         {
    //         }; // wait for OUTB to go LOW.. CK
    //         counter++;
    //         while (digitalRead(OUTA_Pin) == LOW)
    //         {
    //         }; // wait for OUTA to go HIGH
    //         while (digitalRead(OUTB_Pin) == LOW)
    //         {
    //         };         // wait for OUTB to go HIGH
    //         delay(10); // wait for some more time
    //     }

    //     if (counter < 0)
    //         counter = 0;
    //     if (counter > 180)
    //         counter = 180;
    // }
    // SerialUSB.println(counter);

    SerialUSB.println();
}