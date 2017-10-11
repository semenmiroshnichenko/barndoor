#define DEBOUNCING_TIME 30000 //Debouncing Time in microseconds
#define STEPS_PER_UTURN 4096
#define MICROSECONDS_IN_SECOND 1000000
#define TRUE 1
#define FALSE 0
#define ALPHA_AT_THE_START_POSITION_IN_RADIANT 0.17 //Measure this angle on the platform
#define MAX_STEPS_NUM 100000000 //define this
#define START_STEP_NUM 1000000
#define NORMAL_DIRECTION 1
#define REVERT_DIRECTION -1


HardwareTimer recalcTimer(2);
HardwareTimer stepperTimer(3);
HardwareTimer buttonsTimer(4);

byte ledState = 0;
bool blinkingOnOff = true;
volatile unsigned long last_micros;
long stepperDirection = NORMAL_DIRECTION;
byte run = FALSE;

float alphaInRadiant = ALPHA_AT_THE_START_POSITION_IN_RADIANT;
float const deltaAlphaPerSecondInRadiant = 0.000072722; //1 / 240 degrees

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
}

void InitButtonPins()
{
  pinMode(PA15, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);
  pinMode(PB5, INPUT_PULLUP);  
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
  recalcTimer.setPeriod(MICROSECONDS_IN_SECOND); // in microseconds
  recalcTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  recalcTimer.setCompare(TIMER_CH1, 1);  
  recalcTimer.attachCompare1Interrupt(handler_RecalcTimer);
  recalcTimer.refresh();
  recalcTimer.resume();
}

void InitStepperTimer()
{
  stepperTimer.pause();
  stepperTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  stepperTimer.setCompare(TIMER_CH1, 1); 
  stepperTimer.attachCompare1Interrupt(handler_StepperTimer);
  stepperTimer.refresh();
}

void InitButtonsTimer()
{
  buttonsTimer.pause();
  buttonsTimer.setPeriod(DEBOUNCING_TIME); // in microseconds
  buttonsTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  buttonsTimer.setCompare(TIMER_CH1, 1); 
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
  if(stepNumber > MAX_STEPS_NUM || stepNumber <= START_STEP_NUM)
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
  return 60000000 / (3276.8 * sqrt(pow((286 * tan(alphaInRadiant)), 2) + pow(286, 2)) * tan(deltaAlphaPerSecondInRadiant * 60));
}

void handler_RecalcTimer(void) 
{
  if(run)
  {
    toggleLED();
    alphaInRadiant += deltaAlphaPerSecondInRadiant;
    float stepperDelayInUs = getCurrentStepperDelayInUs();
    stepperTimer.pause();
    stepperTimer.setPeriod(stepperDelayInUs); // in microseconds
    stepperTimer.refresh();
    stepperTimer.resume();
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
  alphaInRadiant = ALPHA_AT_THE_START_POSITION_IN_RADIANT;
  stepperDirection = REVERT_DIRECTION;
  stepperTimer.pause();
  stepperTimer.setPeriod(2000); // in microseconds
  stepperTimer.refresh();
  stepperTimer.resume();
}

void ProcessStopPressed()
{
  run = FALSE;
  StopStepperTimerAndResetPins();
}

void StopStepperTimerAndResetPins()
{
  stepperTimer.pause();
  SetStepperPinsToLow();
}

void handler_ButtonsTimer()
{
  static unsigned int lastPA15Status = HIGH;
  static unsigned int lastPB4Status = HIGH;
  static unsigned int lastPB5Status = HIGH;
  int pinPA15Status = digitalRead(PA15);
  int pinPB4Status = digitalRead(PB4);
  int pinPB5Status = digitalRead(PB5);
  if(pinPA15Status != lastPA15Status && pinPA15Status == LOW)
  {
    ProcessStopPressed();
  }
  if(pinPB4Status != lastPB4Status && pinPB4Status == LOW)
  {
    ProcessRevertPressed();
  }
  if(pinPB5Status != lastPB5Status && pinPB5Status == LOW)
  {
    ProcessStartPressed();
  }
  lastPA15Status = pinPA15Status;
  lastPB4Status = pinPB4Status;
  lastPB5Status = pinPB5Status;
}

