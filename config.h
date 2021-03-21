#ifndef __CONFIG_H__
#define __CONFIG_H__

// general options
#define USE_FAST_ACCEL_STEPPER_LIB  (0)   // for faster movements speeds, uses timer1, does not compile on AtTinyCore
#define USE_TIMER1_FOR_STEP_PULSE   (0)   // 1 - gives more time for background operation, but uses timer1
#define SOFT_CHANGE_ENABLE          (1)   // make quarter of revolution with acceleration/deceleration before jump into sync mode
#define SOFT_CHANGE_VALUE           (100)  // divisor walue where soft start applies

#define DIVISOR_STABLE_TIME         (500) // 0.5 sec

// divisor selector pins
#define SELECTOR_BIT0_PIN   (10)
#define SELECTOR_BIT1_PIN   (11)
#define SELECTOR_BIT2_PIN   (12)
#define SELECTOR_BIT3_PIN   (13)

#define SELECTOR0_SEL_PIN   (9)
#define SELECTOR1_SEL_PIN   (8)
#define SELECTOR2_SEL_PIN   (7)

#define SELECTOR_NUMBER     (3)
#define SELECTOR_SEL_ACTIVE (0)
#define SELECTOR_DEBUG      (0)

#define LED_PIN             (0)

#define MODE_SWITCH_PIN     (6)
#define ROTATE_BUTTON_PIN   (5)

// encoder configuration
#define ENC_PULSES_PER_REVOLUTION (500)   // pulsses per revolution
#define ENC_REDUCTOR_RATIO        (4)
#define ENC_MAX_RPS               (10 * ENC_REDUCTOR_RATIO)    // max revolutions per second
#define ENC_PULSES_MULT           (4)

// stepper configuration
#define STEP_PIN            (4)
#define DIR_PIN             (14)
#define STEP_PIN_ACTIVE     (0)
#define DIR_PIN_ACTIVE      (0)

#define MOTOR_STEPS_PER_REVOLUTION  (200)
#define MOTOR_REDUCTOR_RATIO        (1)
#define DRIVER_MICROSTEPS           (8)
#define STEPS_PER_REVOLUTION        (MOTOR_STEPS_PER_REVOLUTION * MOTOR_REDUCTOR_RATIO * DRIVER_MICROSTEPS)

#define NS_PER_SEC                  (1000000000L)
#define CLOCK_PERIOD_NS             (NS_PER_SEC / (F_CPU / 10)) // divide F_CPU by 10 in order to compute in integer, so clock period is 10 times bigger
#define NS_PER_US                   (1000)
#define STEP_PULSE_WIDTH_US         (2)//(1000000 / 2 * (ENC_PULSES_PER_REVOLUTION * ENC_MAX_RPS * ENC_PULSES_MULT))
#define STEP_PULSE_WIDTH_NS         (STEP_PULSE_WIDTH_US * NS_PER_US)
#if (USE_TIMER1_FOR_STEP_PULSE == 1) // TODO: FIXME
#define STEP_PULSE_CLOCKS           (10 * STEP_PULSE_WIDTH_NS / CLOCK_PERIOD_NS) // multiply by 10 in order to fix F_CPU division
#else
#define STEP_PULSE_CLOCKS           (10 * STEP_PULSE_WIDTH_NS / CLOCK_PERIOD_NS)/2 // multiply by 10 in order to fix F_CPU division
#endif

#define STEPPER_SPEED               (4500)
#define STEPPER_ACCEL               (STEPPER_SPEED * 2)

#define SERIAL_PORT_ENABLE    (0)
#define SERIAL_PORT_BAUDRATE  (115200)
#endif //__CONFIG_H__
