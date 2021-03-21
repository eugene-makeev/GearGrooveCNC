#include "config.h"

#if (USE_FAST_ACCEL_STEPPER_LIB == 1)
#include "FastAccelStepper.h"
#else
#include "AccelStepper.h"
#endif

#include "button.h"

#define MIN(a,b) (((a) > (b)) ? (b) : (a))
#define MAX(a,b) (((a) < (b)) ? (b) : (a))

#define GENERAL_ERROR       (-1)

#define EXT_PIN_INT_MASK    ((1 << INT0) | (1 << INT1))
#define STEP_PIN_MASK       (1 << STEP_PIN)
#define TIMER_ENABLE_MASK   (1 << OCIE1A)

enum
{
    MODE_0 = 0, // used for rotation by pressing the button
    MODE_1,     // used for continuous rotation by encoder pulses (4 steps per pulse)
    NUM_MODES
};

// stepper motor
#if (USE_FAST_ACCEL_STEPPER_LIB == 1)
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
#else
// stepper motor
AccelStepper motor(1, STEP_PIN, DIR_PIN);
AccelStepper *stepper = &motor;
#endif


void rotate(void);

// Buttons
Button mode_switch(MODE_SWITCH_PIN);
Button rotate_button(ROTATE_BUTTON_PIN, rotate);

volatile int divisor = 0;     // in case if selector swith is broken, divisor will always remain 0 (no frequency division)
int mode = MODE_1;            // in case if no switch connected, the freq division mode by default
int segment_number = 0;       // number of current segment in MODE_0
volatile int edge_number = 0; // isr variable counting pulses edges

#if (USE_TIMER1_FOR_STEP_PULSE == 1)
void timer1_init()
{
  cli();
  TCCR1A = TCCR1B = 0; 

  OCR1A = STEP_PULSE_CLOCKS;
  TCCR1B = (1 << WGM12);     // CTC mode
  TCCR1B |= (1 << CS10);     // no prescale
  
  //TIMSK1 |= (1 << OCIE1A); // enable isr
  sei();
}
#endif

void setup()
{
  cli();
  // initialize selector ports
  pinMode(SELECTOR_BIT0_PIN, INPUT_PULLUP);
  pinMode(SELECTOR_BIT1_PIN, INPUT_PULLUP);
  pinMode(SELECTOR_BIT2_PIN, INPUT_PULLUP);
  pinMode(SELECTOR_BIT3_PIN, INPUT_PULLUP);

  pinMode(SELECTOR0_SEL_PIN, OUTPUT);
  pinMode(SELECTOR1_SEL_PIN, OUTPUT);
  pinMode(SELECTOR2_SEL_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  // configure motors
#if (USE_FAST_ACCEL_STEPPER_LIB == 1)
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) 
  {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setSpeedInHz(1000);       // the parameter is us/step !!!??
    stepper->setAcceleration(100);
  }
#else
  stepper->setPinsInverted(DIR_PIN_ACTIVE, !STEP_PIN_ACTIVE, !DIR_PIN_ACTIVE);
  stepper->setAcceleration(STEPPER_ACCEL);
  stepper->setSpeed(STEPPER_SPEED);
  stepper->enableOutputs();
#endif

  // set default STEP_PIN state
  digitalWrite(STEP_PIN, !STEP_PIN_ACTIVE);

  // initialize timer 1 for step pulse width
#if (USE_TIMER1_FOR_STEP_PULSE == 1)
  timer1_init();
#endif

  EICRA = (1 << ISC00) | (1 << ISC10);  // sense any change on the INT0 / INT1 pin
    
  // enable interrupts if switch is in MODE1
  mode = digitalRead(MODE_SWITCH_PIN);
  if (mode == MODE_1)
  {
    divisor = get_divisor(false);
#if (SOFT_CHANGE_ENABLE == 1)
    divisor = MAX(SOFT_CHANGE_VALUE, divisor);
#endif
    EIMSK = EXT_PIN_INT_MASK;   // enable INT0/1 interrupt

#if (SOFT_CHANGE_ENABLE == 1)
    divisor = MAX(SOFT_CHANGE_VALUE, divisor);
#endif
  }
  // set default LED_PIN state
  digitalWrite(LED_PIN, mode ? HIGH : LOW);

  sei();
}

int get_selector_pin(int selector)
{
  switch (selector)
  {
    case 0:
      return SELECTOR0_SEL_PIN;
    case 1:
      return SELECTOR1_SEL_PIN;
    case 2:
      return SELECTOR2_SEL_PIN;
  }

  return GENERAL_ERROR;
}

int get_selector_value(int selector)
{
  int selector_pin = get_selector_pin(selector);

  if (selector_pin > GENERAL_ERROR)
  {
    digitalWrite(selector_pin, SELECTOR_SEL_ACTIVE);
  
    int value = digitalRead(SELECTOR_BIT0_PIN);
    value |= digitalRead(SELECTOR_BIT1_PIN) << 1;
    value |= digitalRead(SELECTOR_BIT2_PIN) << 2;
    value |= digitalRead(SELECTOR_BIT3_PIN) << 3;

    digitalWrite(selector_pin, !SELECTOR_SEL_ACTIVE);

    // inverted output, decimal only
    return MIN((value ^ 0xF), 9);
  }
  
  return GENERAL_ERROR;
}

int get_divisor(bool stable)
{
  static int last_dvsr = GENERAL_ERROR;
  static long last_divisor_change = 0;
  
  int dvsr = 0;
  
  for (int idx = SELECTOR_NUMBER; idx > 0; )
  { 
    int sel_value = get_selector_value(idx - 1);
    if (sel_value == GENERAL_ERROR)
    {
      return GENERAL_ERROR;
    }
    else
    {
      dvsr = dvsr * 10 + sel_value;
      idx --;
    }
  }

  if (last_dvsr != dvsr)
  {
    last_dvsr = dvsr;
    last_divisor_change = millis();
  }
  
  if (!stable || (last_divisor_change + DIVISOR_STABLE_TIME < millis()))
  {
    return dvsr;
  }

  return GENERAL_ERROR;
}

void rotate(void)
{
  long position;
  segment_number++;
  
  if (segment_number < divisor)
  {
    position = segment_number * STEPS_PER_REVOLUTION / divisor;
  }
  else
  {
    position = STEPS_PER_REVOLUTION;
    segment_number = 0;
  }

  stepper->moveTo(position);
  while(stepper->isRunning())
  {
#if (USE_FAST_ACCEL_STEPPER_LIB == 0)
    stepper->run();
#endif
  }

  // reset current position
  if (position >= STEPS_PER_REVOLUTION)
  {
    stepper->setCurrentPosition(0);
  }
}

#if (SOFT_CHANGE_ENABLE == 1)
void soft_change(int from, int to)
{
  int delay_ms = 1;
  int slow_steps = (from > to) ? 0 : SOFT_CHANGE_VALUE;
  
  while(from != to)
  {
    if (from > to)
    {
      if ((from - to) <= SOFT_CHANGE_VALUE)
      {
        slow_steps = SOFT_CHANGE_VALUE;
      }

      from--;
    }
    else
    {
      from++;
    }

    cli();
    divisor = from;
    sei();
    
    if (slow_steps)
    {
      delay_ms = 10;
      slow_steps --;
    }
    else
    {
      delay_ms = 1;
    }
    delay(delay_ms);
  } 
}
#endif //(SOFT_CHANGE_ENABLE == 1)

void loop()
{
  // DIVISOR_STABLE_TIME is time for stabilize divisor value, during this time 
  // if value has recently changed, GENERAL_ERROR status would be returned
  int new_divisor = get_divisor(true);
  if ((new_divisor > GENERAL_ERROR) && new_divisor != divisor)
  {
    segment_number = 0;
    stepper->setCurrentPosition(0);
    cli();
    edge_number = 0;
    sei();
    
#if (SOFT_CHANGE_ENABLE == 1)
    if (mode == MODE_1)
    {
      soft_change(divisor, new_divisor);
    }
    else
#endif //(SOFT_CHANGE_ENABLE == 1)
    {
      divisor = new_divisor;
    }
  }
  
  // if mode has changed
  if (mode_switch.check())
  {
    mode = mode_switch.getState();
    // set default LED_PIN state
    digitalWrite(LED_PIN, mode ? HIGH : LOW);

    cli();
    edge_number = 0;
    sei();
    
    if (mode == MODE_1)
    {
      #if (SOFT_CHANGE_ENABLE == 1)
      // acceleration
      if (divisor < SOFT_CHANGE_VALUE)
      {
        int temp_divisor = divisor;
        cli();
        divisor = SOFT_CHANGE_VALUE;
        sei();
        EIMSK = EXT_PIN_INT_MASK;   // enable INT0/1 interrupt
        soft_change(SOFT_CHANGE_VALUE, temp_divisor);
      }
      else
      #endif
      {
         EIMSK = EXT_PIN_INT_MASK;   // enable INT0/1 interrupt
      }
    }
    else
    {
      #if (SOFT_CHANGE_ENABLE == 1)
      // deceleration
      if (divisor < SOFT_CHANGE_VALUE)
      {
        soft_change(divisor, SOFT_CHANGE_VALUE);
      }
      #endif

      segment_number = 0;
      stepper->setCurrentPosition(0);
      
      EIMSK = 0;   // disable INT0/1 interrupt
    }
  }
  
  if (mode == MODE_0)
  {
    rotate_button.check();
  }
}

ISR(INT1_vect, ISR_ALIASOF(INT0_vect));

// TODO: modify the code in order to use proper variables addresses for divisor and edge_number
// TODO: modify the code in order to use defibed constant for STEP_PULSE_CLOCKS
ISR(INT0_vect, ISR_NAKED)
{
//  edge_number ++;
//  if (edge_number >= divisor)
//  {
//#if (STEP_PIN_ACTIVE == 1)
//      PORTD |= STEP_PIN_MASK;
//#else
//      PORTD &= ~STEP_PIN_MASK;
//#endif //(STEP_PIN_ACTIVE == 0)
//      edge_number = 0;
//#if (USE_TIMER1_FOR_STEP_PULSE == 0)
//      TIMSK0 = (1 << OCIE0A); // enable interrupt
//#else
//      for(byte i = 0; i < STEP_PULSE_CLOCKS; i++)
//      {
//      }
//#endif
//  }

  asm volatile(
  // store registers
  "   push  r18        \n"
  "   in    r18, 0x3f  \n" // status register
  "   push  r18        \n"
  "   push  r19        \n" 
  "   push  r24        \n" 
  "   push  r25        \n" 
  
//edge_number ++;
  "   lds r24, 0x0128  \n" // load edge_number
  "   lds r25, 0x0129  \n"
  "   adiw  r24, 0x01  \n"

//if (edge_number >= divisor)
  "   lds r18, 0x012C  \n" // load divisor
  "   lds r19, 0x012D  \n"
  "   cp  r24, r18     \n" 
  "   cpc r25, r19     \n" 
  "   brlt  int0_exit  \n"

//PORTD ^= STEP_PIN_MASK;
  "   in  r18, 0x0b    \n"
#if (STEP_PIN_ACTIVE == 1)
  "   ldi r19, 0x10    \n"
  "   or r18, r19      \n"
#else
  "   ldi r19, 0xEF    \n"
  "   and r18, r19     \n"
#endif //(STEP_PIN_ACTIVE == 0)
  "   out 0x0b, r18    \n"
  
//edge_number = 0;
  "   eor r24, r24     \n"
  "   eor r25, r25     \n" 

#if (USE_TIMER1_FOR_STEP_PULSE == 0)
  "   ldi r18, 0x10    \n" // STEP_PULSE_CLOCKS
  "delay_loop:   \n"
  "   subi  r18, 0x01  \n"
  "   brne  delay_loop \n"
  
// PORTD ^= STEP_PIN_MASK;    
  "   in  r18, 0x0b    \n"
  "   ldi r19, 0x10    \n"
  "   eor r18, r19     \n"
  "   out 0x0b, r18    \n"
#else
//TCNT1 = 0;
  "   sts 0x85, r24       \n" // TCNT1H
  "   sts 0x84, r24       \n" // TCNT1L
//TIMSK1 = (1 << OCIE0A); // enable interrupt
  "   ldi r18, 0x02       \n"
  "   sts 0x6F, r18       \n"
#endif //(USE_TIMER1_FOR_STEP_PULSE == 0)

  "int0_exit:             \n"
  "   sts 0x0129, r25     \n" // store edge_number
  "   sts 0x0128, r24     \n"

  // restore registers 
  "   pop r25             \n"
  "   pop r24             \n"
  "   pop r19             \n"
  "   pop r18             \n"
  "   out 0x3f, r18       \n"
  "   pop r18             \n"
  "   reti                \n");
}

#if (USE_TIMER1_FOR_STEP_PULSE == 1)
ISR(TIMER1_COMPA_vect, ISR_NAKED)
{
// PORTD ^= STEP_PIN_MASK;    
// TIMSK1 = 0; // disable interrupt

  asm volatile(
  // store registers
  "   push r18         \n"
  "   in   r18, 0x3f   \n" // status register
  "   push r18         \n"
  "   push r19         \n" 
  
// PORTD ^= STEP_PIN_MASK;    
  "   in  r18, 0x0b    \n"
  "   ldi r19, 0x10    \n"
  "   eor r18, r19     \n"
  "   out 0x0b, r18    \n"
  
// TIMSK1 = 0; // disable interrupt   
  "   eor r18, r18     \n"
  "   sts 0x6F, r18    \n"
    
  // restore registers 
  "   pop r19          \n"
  "   pop r18          \n"
  "   out 0x3f, r18    \n" // status register
  "   pop r18          \n"
  "   reti             \n");
}
#endif
