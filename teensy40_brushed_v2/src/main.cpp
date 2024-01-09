#include <Arduino.h>
#include "hardware_pins.h"
#include "driver_motor.h"
#include "model_encoder.h"
#include "model_sensor.h"
#include "motor.h"

#define OUTA_Pin ENCPIN1_1
#define OUTB_Pin ENCPIN1_2

model_encoder enc1;
model_sensor cur1;
driver_motor mot1;

model_encoder enc2;
model_sensor cur2;
driver_motor mot2;

model_encoder enc3;
model_sensor cur3;
driver_motor mot3;

int counter = 0; // Ensure counter is initialized

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

void lim1ISR();
void lim2ISR();
void lim3ISR();
void lim4ISR();
void lim5ISR();
void lim6ISR();

void setup()
{
    SerialUSB.begin(115200);

    // SENSORS
    // 43000 for wrist pitch
    enc1.initialize_encoder(0, 0, 43000, 1);
    cur1.initialize_sensor(CURRENT_SENSE_A);

    enc2.initialize_encoder(0, 0, 43000, 2);
    cur2.initialize_sensor(CURRENT_SENSE_B);

    enc3.initialize_encoder(0, 0, 43000, 3);
    cur3.initialize_sensor(CURRENT_SENSE_C);

    SerialUSB.println("Done Setup");

    // Initialize Pins
    pinMode(PWMPIN1, OUTPUT);
    pinMode(DIRPIN1, OUTPUT);
    pinMode(nSLEEP1, OUTPUT);
    pinMode(PWMPIN2, OUTPUT);
    pinMode(DIRPIN2, OUTPUT);
    pinMode(nSLEEP2, OUTPUT);
    pinMode(PWMPIN3, OUTPUT);
    pinMode(DIRPIN3, OUTPUT);
    pinMode(nSLEEP3, OUTPUT);

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
}

void loop()
{
    printf("LIM_1: %d, LIM_2: %d, LIM_3: %d, LIM_4: %d, LIM_5: %d, LIM_6: %d\n",
           lim1_state, lim2_state, lim3_state, lim4_state, lim5_state, lim6_state);

    delay(100);
    // DUMB TEST MOTOR
    // analogWrite(PWMPIN1, 50);
    // delay(1000);
    // analogWrite(PWMPIN1, 125);
    // delay(1000);
    // SerialUSB.println(digitalRead(ENCPIN1_1));

    // TEST CURRENT SENSOR
    // cur1.read_sensor_value();
    // cur2.read_sensor_value();
    // cur3.read_sensor_value();
    // float cur1_current = cur1.getCurrent();
    // float cur2_current = cur2.getCurrent();
    // float cur3_current = cur3.getCurrent();
    // SerialUSB.printf("cur1_current: %f, cur2_current: %f, cur3_current: %f ", cur1_current, cur2_current, cur3_current);

    // TEST ENCODER
    // enc1.read_encoder_angle();
    // enc2.read_encoder_angle();
    // enc3.read_encoder_angle();
    // float enc1_angle = (enc1.getAngle());
    // float enc2_angle = (enc2.getAngle());
    // float enc3_angle = (enc3.getAngle());
    // SerialUSB.printf("enc1_angle: %f, enc2_angle: %f, enc3_angle: %f", enc1_angle, enc2_angle, enc3_angle);

    // SerialUSB.println();

    // Test gravity compensation
    // double output;
    // current_rotation->setAngleRad(current_angle / 360 * 2 * PI);
    // maintainStateProto(*current_rotation, &output);
    // printf("current output: %f\r\n", output);
    // printf("Current voltage: %f\n", output);
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

    // delay(10);
    // analogWrite(PWM1, 200);
    // delay(1000);
    // analogWrite(PWM1, 50);
    // delay(1000);

    // DUMB VERSION
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
