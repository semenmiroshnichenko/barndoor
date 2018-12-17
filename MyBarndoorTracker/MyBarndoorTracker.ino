#define DEBOUNCING_TIME 30000 //Debouncing Time in microseconds
#define STEPS_PER_UTURN 4076
#define MICROSECONDS_IN_SECOND 1000000
#define ARCSECONDS_IN_RADIANT 206264.8
#define TRUE 1
#define FALSE 0
#define RECALC_EACH_N_SEC 1

//correction data:
#define LINEAR_ERROR_SLOPE_ARCSEC_PER_SECOND 0.159828870656 //positive means too fast
#define PERIODIC_ERROR_ANGULAR_FREQUENCY_W 0.0542207201
#define PERIODIC_ERROR_PHASE_P 0.0299584338
#define PERIODIC_ERROR_AMPLITUDE_A 18.1483706428

//#define __GUNTRAM 1

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
#define START_STELLAR_MODE_BUTTON PB5
#define REWIND_BUTTON PB4
#define FULLSPEED_START_BUTTON PA10
#define LIMIT_SWITCH PA9
#define START_MOON_MODE_BUTTON PA8
#define TEST_PIN_1 PA7

HardwareTimer recalcTimer(2);
HardwareTimer stepperTimer(3);
HardwareTimer buttonsTimer(4);

byte ledState = 0;
bool blinkingOnOff = true;
volatile unsigned long last_micros;
long stepperDirection = NORMAL_DIRECTION;
byte run = FALSE;
int secondsFromStart = 0;
byte test_Pin_State = 0;

enum trackingMode
{
  stellar = 1,
  moon = 2
};

trackingMode actualTrackingMode = stellar;

float alphaInRadiant = ALPHA_AT_THE_START_POSITION_IN_RADIANT;
float const deltaAlphaPerSecondInArcSecondsStellar = 15.0410500749;
float const deltaAlphaPerSecondInRadiantStellar = deltaAlphaPerSecondInArcSecondsStellar / ARCSECONDS_IN_RADIANT;

float const deltaAlphaPerSecondInArcSecondsMoon = 14.49;
float const deltaAlphaPerSecondInRadiantMoon = deltaAlphaPerSecondInArcSecondsMoon / ARCSECONDS_IN_RADIANT;

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
  InitTestPin();
  
  //Serial.begin(9600);
  //Serial.write("Init");
}

void InitButtonPins()
{
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(REWIND_BUTTON, INPUT_PULLUP);
  pinMode(START_STELLAR_MODE_BUTTON, INPUT_PULLUP);  
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(FULLSPEED_START_BUTTON, INPUT_PULLUP);
  pinMode(START_MOON_MODE_BUTTON, INPUT_PULLUP);
}

void InitStepperPins()
{
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
}

void InitTestPin()
{
  pinMode(TEST_PIN_1, OUTPUT);
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
  ToggleTestPin();
}

void ToggleTestPin()
{
  switch(test_Pin_State & 1)
  {
    case 0: gpio_write_bit(GPIOA, 7, LOW); break;
    case 1: gpio_write_bit(GPIOA, 7, HIGH); break;
  }
  test_Pin_State++;
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

float getCurrentDeltaAlphaPerSecondInArcSeconds()
{
  switch(actualTrackingMode)
  {
    case stellar: return deltaAlphaPerSecondInArcSecondsStellar;
    case moon: return deltaAlphaPerSecondInArcSecondsMoon;
  }
}

float getCurrentDeltaAlphaPerSecondInRadiant()
{
  switch(actualTrackingMode)
  {
    case stellar: return deltaAlphaPerSecondInRadiantStellar;
    case moon: return deltaAlphaPerSecondInRadiantMoon;
  }
}


float getCurrentStepperDelayInUs()
{
  
  #if defined (__GUNTRAM)
    return 60000000 / (3276.8 * sqrt(pow((286 * tan(alphaInRadiant)), 2) + pow(286, 2)) * tan(getCurrentDeltaAlphaPerSecondInRadiant() * 60));
  #else
    float const delayPreConstant = (float)MICROSECONDS_IN_SECOND / (float)((STEPS_PER_UTURN / 2) * (476));
    float const deltaAlphaPerSecondInArcSecondsStellarCorrected = 
                getCurrentDeltaAlphaPerSecondInArcSeconds() 
                - LINEAR_ERROR_SLOPE_ARCSEC_PER_SECOND
                // first derivate of periodic error tracing given by linearizeIt.py
                - PERIODIC_ERROR_AMPLITUDE_A * PERIODIC_ERROR_ANGULAR_FREQUENCY_W 
                    * cos(PERIODIC_ERROR_ANGULAR_FREQUENCY_W * secondsFromStart + PERIODIC_ERROR_PHASE_P);
    float const deltaAlphaPerSecondInRadiantStellarCorrected = deltaAlphaPerSecondInArcSecondsStellarCorrected / ARCSECONDS_IN_RADIANT;
    
    return delayPreConstant / (sin(alphaInRadiant / 2 + deltaAlphaPerSecondInRadiantStellarCorrected / 2) - sin(alphaInRadiant / 2));
  #endif
  //return 1200; //max speed
}

void handler_RecalcTimer(void) 
{
  if(run)
  {
    toggleLED();
    alphaInRadiant += getCurrentDeltaAlphaPerSecondInRadiant() * RECALC_EACH_N_SEC;
    secondsFromStart += RECALC_EACH_N_SEC;
    float stepperDelayInUs = getCurrentStepperDelayInUs();
    stepperTimer.setPeriod(stepperDelayInUs); // in microseconds
  }
}

void handler_StepperTimer(void)
{
  DoNextStep();
}

void ProcessStartStellarModePressed()
{
  actualTrackingMode = stellar;
  StartTracker();
}

void ProcessStartMoonModePressed()
{
  actualTrackingMode = moon;
  StartTracker();
}

void StartTracker()
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
  #if defined (__GUNTRAM)
  #else
  
  if(digitalRead(LIMIT_SWITCH) == HIGH)
    return;
  #endif 
  alphaInRadiant = ALPHA_AT_THE_START_POSITION_IN_RADIANT;
  secondsFromStart = 0;
  stepNumber = START_STEP_NUM;
  stepperDirection = REVERT_DIRECTION;
  stepperTimer.pause();
  #if defined (__GUNTRAM)
    stepperTimer.setPeriod(1500); // in microseconds
  #else
    stepperTimer.setPeriod(1300); // in microseconds
  #endif
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
  stepperTimer.setPeriod(1300); // in microseconds
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
  static unsigned int lastSTART_STELLAR_MODE_BUTTONStatus = HIGH;
  static unsigned int lastLIMIT_SWITCHStatus = LOW; //normally closed switch
  static unsigned int lastFULLSPEED_START_BUTTONStatus = HIGH;
  static unsigned int lastSTART_MOON_MODE_BUTTONStatus = HIGH;
  
  int pinSTOP_BUTTONStatus = digitalRead(STOP_BUTTON);
  int pinREWIND_BUTTONStatus = digitalRead(REWIND_BUTTON);
  int pinSTART_BUTTONStatus = digitalRead(START_STELLAR_MODE_BUTTON);
  int pinLIMIT_SWITCHStatus = digitalRead(LIMIT_SWITCH);
  int pinFULLSPEED_START_BUTTONStatus = digitalRead(FULLSPEED_START_BUTTON);
  int pinSTART_MOON_MODE_BUTTONStatus = digitalRead(START_MOON_MODE_BUTTON);
  
  if(pinSTOP_BUTTONStatus != lastSTOP_BUTTONStatus && pinSTOP_BUTTONStatus == LOW)
  {
    ProcessStopPressed();
  }
  if(pinREWIND_BUTTONStatus != lastREWIND_BUTTONStatus && pinREWIND_BUTTONStatus == LOW)
  {
    ProcessRevertPressed();
  }
  if(pinSTART_BUTTONStatus != lastSTART_STELLAR_MODE_BUTTONStatus && pinSTART_BUTTONStatus == LOW)
  {
    ProcessStartStellarModePressed();
  }
  if(pinLIMIT_SWITCHStatus != lastLIMIT_SWITCHStatus && pinLIMIT_SWITCHStatus == HIGH)
  {
    ProcessLimitSwitchClosed();
  }
  if(pinFULLSPEED_START_BUTTONStatus != lastFULLSPEED_START_BUTTONStatus && pinFULLSPEED_START_BUTTONStatus == LOW)
  {
    ProcessStartFullSpeedPressed();
  }
  if(pinSTART_MOON_MODE_BUTTONStatus != lastSTART_MOON_MODE_BUTTONStatus && pinSTART_MOON_MODE_BUTTONStatus == LOW)
  {
    ProcessStartMoonModePressed();
  }

  lastSTOP_BUTTONStatus = pinSTOP_BUTTONStatus;
  lastREWIND_BUTTONStatus = pinREWIND_BUTTONStatus;
  lastSTART_STELLAR_MODE_BUTTONStatus = pinSTART_BUTTONStatus;
  lastLIMIT_SWITCHStatus = pinLIMIT_SWITCHStatus;
  lastFULLSPEED_START_BUTTONStatus = pinFULLSPEED_START_BUTTONStatus;
  lastSTART_MOON_MODE_BUTTONStatus = pinSTART_MOON_MODE_BUTTONStatus;
}

