#include <DRV8834.h>
#include <A4988.h>
#include <SyncDriver.h>
#include <BasicStepperDriver.h>
#include <DRV8880.h>
#include <MultiDriver.h>
#include <DRV8825.h>

// using a 200-step motor (most common)
#define MOTOR_STEPS 200

// configure the pins connected
#define DIR 8
#define STEP 9
#define MS1 10
#define MS2 11
#define MS3 12
A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

void setup() {
    // Set target motor RPM to 1RPM and microstepping to 1 (full step mode)
    stepper.begin(15, 1);
}

void loop() {
    // Tell motor to rotate 360 degrees. That's it.
    stepper.rotate(360);
}
