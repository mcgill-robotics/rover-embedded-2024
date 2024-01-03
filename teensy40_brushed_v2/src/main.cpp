#include <Arduino.h>
#include "hardware_pins.h"
#include "driver_motor.h"
#include "model_encoder.h"
#include "model_sensor.h"
#include "motor.h"
#define NSLEEP1 9
#define DIR1 16
#define PWM1 13

model_encoder enc1;
model_sensor cur1;
driver_motor mot1;
// Define your pins numbers here
const int OUTA_Pin = 3; // Example pin numbers, change as per your hardware
const int OUTB_Pin = 4;
int counter = 0; // Ensure counter is initialized

Rotation2d *current_rotation;

void setup()
{
    // SerialUSB.begin(115200);
    enc1.initialize_encoder(0, 0, 43000, 1);
    cur1.initialize_sensor(CURRENT_SENSE_A);
    SerialUSB.println("Done Setup");

    // pinMode(OUTA_Pin, INPUT);
    // pinMode(OUTB_Pin, INPUT);
    pinMode(NSLEEP1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    digitalWrite(NSLEEP1, HIGH);
    digitalWrite(DIR1, HIGH);

    // Initialize Motor
    current_rotation = Rotation2d::getRotationFromDeg(0);
}

void loop()
{
    // Poll encoder
    enc1.read_encoder_angle();

    // INFO
    // SerialUSB.print("Current: ");
    // SerialUSB.print(cur1.getCurrent());
    SerialUSB.print(" Position: ");
    double current_angle = (enc1.getAngle() - 90);
    SerialUSB.println(current_angle);
    // analogWrite(PWM1, 50);

    // Test gravity compensation
    double output;
    current_rotation->setAngleRad(current_angle / 360 * 2 * PI);
    maintainStateProto(*current_rotation, &output);
    printf("current output: %f\r\n", output);
    printf("Current voltage: %f\n", output);
    if (output < 0)
    {
        digitalWrite(DIR1, HIGH);
    }
    else
    {
        digitalWrite(DIR1, LOW);
    }
    int analog_write_output = (output / 24) * 255;
    printf("Current analog output: %d\n", analog_write_output);
    analogWrite(PWM1, analog_write_output);

    // DUMB TEST MOTOR
    

    // delay(10);
    // analogWrite(PWM1, 200);
    // delay(1000);
    // analogWrite(PWM1, 50);
    // delay(1000);

    // DUMB VERSION
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