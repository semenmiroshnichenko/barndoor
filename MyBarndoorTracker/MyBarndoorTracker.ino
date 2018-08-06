#define DEBOUNCING_TIME 30000 //Debouncing Time in microseconds
#define STEPS_PER_UTURN 4076
#define MICROSECONDS_IN_SECOND 1000000
#define TRUE 1
#define FALSE 0
#define RECALC_EACH_N_SEC 1

#if defined (__GUNTRAM)
  #define ALPHA_AT_THE_START_POSITION_IN_RADIANT -0.055885802 //Guntam
#else
  #define ALPHA_AT_THE_START_POSITION_IN_RADIANT 0.0688564893 //my
#endif

#define MAX_STEPS_NUM 100000000 //define this
#define START_STEP_NUM 1000000
#define NORMAL_DIRECTION 1
#define REVERT_DIRECTION -1

// Keys and switches
#define STOP_BUTTON PA15
#define START_BUTTON PB5
#define REWIND_BUTTON PB4
#define FULLSPEED_START_BUTTON PA10
#define LIMIT_SWITCH PA9

HardwareTimer recalcTimer(2);
HardwareTimer stepperTimer(3);
HardwareTimer buttonsTimer(4);

byte ledState = 0;
bool blinkingOnOff = true;
volatile unsigned long last_micros;
long stepperDirection = NORMAL_DIRECTION;
byte run = FALSE;

float alphaInRadiant = ALPHA_AT_THE_START_POSITION_IN_RADIANT;
float const deltaAlphaPerSecondInRadiant = 0.000072921;


long stepNumber = START_STEP_NUM;
byte Seq[8][4] = {{1,0,0,1},
       {1,0,0,0},
       {1,1,0,0},
       {0,1,0,0},
       {0,1,1,0},
       {0,0,1,0},
       {0,0,1,1},
       {0,0,0,1}};


void setup() {
  pinMode(PC13, OUTPUT);
  gpio_write_bit(GPIOC, 13, HIGH);

  InitStepperPins();
  InitButtonPins();
  InitRecalcTimer();
  InitStepperTimer();
  InitButtonsTimer();
  //Serial.begin(9600);
  //Serial.write("Init");
}

void InitButtonPins()
{
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(REWIND_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);  
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(FULLSPEED_START_BUTTON, INPUT_PULLUP);
}

void InitStepperPins()
{
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
}

void InitRecalcTimer()
{
  recalcTimer.pause();
  recalcTimer.setPeriod(RECALC_EACH_N_SEC * MICROSECONDS_IN_SECOND); // in microseconds
  recalcTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  recalcTimer.setCompare(TIMER_CH2, 1);  
  recalcTimer.attachCompare1Interrupt(handler_RecalcTimer);
  recalcTimer.refresh();
  recalcTimer.resume();
}

void InitStepperTimer()
{
  stepperTimer.pause();
  stepperTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  stepperTimer.setCompare(TIMER_CH3, 1); 
  stepperTimer.attachCompare1Interrupt(handler_StepperTimer);
  stepperTimer.refresh();
}

void InitButtonsTimer()
{
  buttonsTimer.pause();
  buttonsTimer.setPeriod(DEBOUNCING_TIME); // in microseconds
  buttonsTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  buttonsTimer.setCompare(TIMER_CH4, 1); 
  buttonsTimer.attachCompare1Interrupt(handler_ButtonsTimer);
  buttonsTimer.refresh();
  buttonsTimer.resume();
}

void DoNextStep()
{
  gpio_write_bit(GPIOB, 6, Seq[stepNumber % 8][0]);
  gpio_write_bit(GPIOB, 7, Seq[stepNumber % 8][1]);
  gpio_write_bit(GPIOB, 8, Seq[stepNumber % 8][2]);
  gpio_write_bit(GPIOB, 9, Seq[stepNumber % 8][3]);
  
  stepNumber += stepperDirection;
  if(stepNumber > MAX_STEPS_NUM)
    StopStepperTimerAndResetPins();
}

void SetStepperPinsToLow()
{
  gpio_write_bit(GPIOB, 6, 0);
  gpio_write_bit(GPIOB, 7, 0);
  gpio_write_bit(GPIOB, 8, 0);
  gpio_write_bit(GPIOB, 9, 0);
}

// the loop function runs over and over again forever
void loop() {

}

void toggleLED()
{
  if(!blinkingOnOff)
    return;
  switch(ledState & 1)
  {
    case 0: gpio_write_bit(GPIOC, 13, LOW); break;
    case 1: gpio_write_bit(GPIOC, 13, HIGH); break;
  }
  ledState++;
}

float getCurrentStepperDelayInUs()
{
  #if defined (__GUNTRAM)
    return 60000000 / (3276.8 * sqrt(pow((286 * tan(alphaInRadiant)), 2) + pow(286, 2)) * tan(deltaAlphaPerSecondInRadiant * 60));
  #else
    float const delayPreConstant = (float)MICROSECONDS_IN_SECOND / (float)((STEPS_PER_UTURN / 2) * (476 - 6.33));
    return delayPreConstant / (sin(alphaInRadiant / 2 + deltaAlphaPerSecondInRadiant / 2) - sin(alphaInRadiant / 2));
  #endif
  //return 1200; //max speed
}

void handler_RecalcTimer(void) 
{
  if(run)
  {
    toggleLED();
    alphaInRadiant += deltaAlphaPerSecondInRadiant * RECALC_EACH_N_SEC;
    float stepperDelayInUs = getCurrentStepperDelayInUs();
    stepperTimer.setPeriod(stepperDelayInUs); // in microseconds
  }
}

void handler_StepperTimer(void)
{
  DoNextStep();
}

void ProcessStartPressed()
{
  run = TRUE;
  stepperDirection = NORMAL_DIRECTION;
  stepperTimer.pause();
  stepperTimer.setPeriod(getCurrentStepperDelayInUs()); // in microseconds
  stepperTimer.refresh();
  stepperTimer.resume();
}

void ProcessRevertPressed()
{
  run = FALSE;
  if(digitalRead(LIMIT_SWITCH) == HIGH)
    return;
   
  alphaInRadiant = ALPHA_AT_THE_START_POSITION_IN_RADIANT;
  stepperDirection = REVERT_DIRECTION;
  stepperTimer.pause();
  stepperTimer.setPeriod(1200); // in microseconds
  stepperTimer.refresh();
  stepperTimer.resume();
}

void ProcessStopPressed()
{
  run = FALSE;
  StopStepperTimerAndResetPins();
}

void ProcessStartFullSpeedPressed()
{
  run = FALSE;
  stepperDirection = NORMAL_DIRECTION;
  stepperTimer.pause();
  stepperTimer.setPeriod(1200); // in microseconds
  stepperTimer.refresh();
  stepperTimer.resume();
}

void StopStepperTimerAndResetPins()
{
  stepperTimer.pause();
  SetStepperPinsToLow();
}

void ProcessLimitSwitchClosed()
{
  // stop rewind
  run = FALSE;
  StopStepperTimerAndResetPins();
  
}

void handler_ButtonsTimer()
{
  static unsigned int lastSTOP_BUTTONStatus = HIGH;
  static unsigned int lastREWIND_BUTTONStatus = HIGH;
  static unsigned int lastSTART_BUTTONStatus = HIGH;
  static unsigned int lastLIMIT_SWITCHStatus = LOW; //normally closed switch
  static unsigned int lastFULLSPEED_START_BUTTONStatus = HIGH;
  
  int pinSTOP_BUTTONStatus = digitalRead(STOP_BUTTON);
  int pinREWIND_BUTTONStatus = digitalRead(REWIND_BUTTON);
  int pinSTART_BUTTONStatus = digitalRead(START_BUTTON);
  int pinLIMIT_SWITCHStatus = digitalRead(LIMIT_SWITCH);
  int pinFULLSPEED_START_BUTTONStatus = digitalRead(FULLSPEED_START_BUTTON);
  
  if(pinSTOP_BUTTONStatus != lastSTOP_BUTTONStatus && pinSTOP_BUTTONStatus == LOW)
  {
    ProcessStopPressed();
  }
  if(pinREWIND_BUTTONStatus != lastREWIND_BUTTONStatus && pinREWIND_BUTTONStatus == LOW)
  {
    ProcessRevertPressed();
  }
  if(pinSTART_BUTTONStatus != lastSTART_BUTTONStatus && pinSTART_BUTTONStatus == LOW)
  {
    ProcessStartPressed();
  }
  if(pinLIMIT_SWITCHStatus != lastLIMIT_SWITCHStatus && pinLIMIT_SWITCHStatus == HIGH)
  {
    ProcessLimitSwitchClosed();
  }
  if(pinFULLSPEED_START_BUTTONStatus != lastFULLSPEED_START_BUTTONStatus && pinFULLSPEED_START_BUTTONStatus == LOW)
  {
    ProcessStartFullSpeedPressed();
  }
  
  lastSTOP_BUTTONStatus = pinSTOP_BUTTONStatus;
  lastREWIND_BUTTONStatus = pinREWIND_BUTTONStatus;
  lastSTART_BUTTONStatus = pinSTART_BUTTONStatus;
  lastLIMIT_SWITCHStatus = pinLIMIT_SWITCHStatus;
  lastFULLSPEED_START_BUTTONStatus = pinFULLSPEED_START_BUTTONStatus;
}

