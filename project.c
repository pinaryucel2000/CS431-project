#define MBED_CONF_MBED_TRACE_ENABLE 1
#define MBED_TRACE_MAX_LEVEL TRACE_LEVEL_INFO
#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP  "cs431"
#include "stm32f413h_discovery_lcd.h"

typedef void (*func_t)(void);

// motor control pins
PwmOut in1(p21);
PwmOut in2(p22);
PwmOut in3(p23);
PwmOut in4(p24);
// end motor control pins

// ultrasonic sensor emulator class, do not modify
class Ultrasonic
{
  protected:
    AnalogIn* ain;
    Timeout* delayer;
    func_t echo_isr;
  public:
    static const int SPEED_OF_SOUND = 3350;
    Ultrasonic(PinName pin, func_t echo_cb)
    {
      this->ain = new AnalogIn(pin);
      this->delayer = new Timeout();
      this->echo_isr = echo_cb;
    }
    ~Ultrasonic()
    {
      delete ain;
      delete delayer;
    }
    void trigger()
    {
      if(this->echo_isr != NULL)
      {
        const float delay = (20.0+this->ain->read()*400.0)/SPEED_OF_SOUND;
        tr_debug("Ultrasonic: emulating %.0fus delay\n", delay*1e6);
        this->delayer->attach_us(this->echo_isr, delay*1e6);
      }
    }
};

// simulation variables, do not modify
Ticker lcd_ticker;
// end simulation variables

// lcd drawing task that emulates the robot, do not modify
#define MOTOR_ACTIVATION_POWER 0.01
#define ROBOT_HEAD_TO_TAIL_PX  8
#define ROBOT_SPEED_MULTIPLIER 2
#define ROBOT_TURN_MULTIPLIER  0.1
#define MAP MOTOR_ACTIVATION_POWER
void lcd_draw_task()
{
  
  // simulation variables, do not modify
  static float tail_x=BSP_LCD_GetXSize()/2;
  static float tail_y=BSP_LCD_GetYSize()/2;
  static float head_x=BSP_LCD_GetXSize()/2;
  static float head_y=BSP_LCD_GetYSize()/2;
  static float dir = 0;
  static float mleft = 0.0f;
  static float mright = 0.0f;
  
  // clear LCD
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(round(tail_x), round(tail_y), 3);
  BSP_LCD_FillCircle(round(head_x), round(head_y), 3);
  
  // error checking
  if(in1 >= MAP && in2 >= MAP)
  {
    tr_err("Invalid power feed to motors: +in1 & +in2");
    return;
  }
  if(in3 >= MAP && in4 >= MAP)
  {
    tr_err("Invalid power feed to motors: +in3 & +in4");
    return;
  }
  
  // combining inputs into direction
  if(in1 >= MAP)
  {
    mleft = in1;
  }
  else if(in2 >= MAP)
  {
    mleft = -in2;
  }
  else
  {
    mleft = 0.0;
  }

  if(in3 >= MAP)
  {
    mright = in3;
  }
  else if(in4 >= MAP)
  {
    mright = -in4;
  }
  else
  {
    mright = 0.0;
  }
  
  const float finalpow = mleft + mright;
  const float powdiff = mleft - mright;
  // step
  tail_x += sin(dir)*finalpow*2;
  tail_y += cos(dir)*finalpow*2;     
  head_x = tail_x + sin(dir)*ROBOT_HEAD_TO_TAIL_PX;
  head_y = tail_y + cos(dir)*ROBOT_HEAD_TO_TAIL_PX;
  // turn
  dir += powdiff*ROBOT_TURN_MULTIPLIER;
  tr_debug("mright, mleft = [%.2f, %.2f]; finalpow, powdiff = [%.2f, %.2f]; dir = %.2f", mright, mleft, finalpow, powdiff, dir*180/3.14);

  // drawing
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_FillCircle(round(tail_x), round(tail_y), 3);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_FillCircle(round(head_x), round(head_y), 3);
}

// splitted wait function which allows for lcd drawing simulation and ultrasonic sensor emulation to work
// do not use bare wait_ms and instead use this function wherever you may need wait.
// Hint: you should only need this function in your main loop to avoid browser crash. 
// You should not need any busy waits like this!!
void splitted_wait_ms(int delay_ms)
{
  static Timer internalTimer;
  internalTimer.start();
  internalTimer.reset();
  while(internalTimer.read_ms()<delay_ms)
  {
    wait_ms(1);
  }
}

void robot_emulator_init()
{
  mbed_trace_init();     // initialize the trace library
  BSP_LCD_Init();
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  lcd_ticker.attach(lcd_draw_task, 0.2);
  printf("Speed of sound is %d m/s due to JS engine limitations\n", Ultrasonic::SPEED_OF_SOUND);  
}

// USER GLOBAL CODE SPACE BEGIN
// your codes should go in here

#define SPEED_1 0.3
#define SPEED_2 0.6
#define MOVEMENT_DURATION_S 3.0
#define ULTRASONIC_PERIOD_S 0.5
#define SAFE_S 0.05
#define BLUETOOTH_S 0.001

// global variables
float speed;
int only_backwards = 0;
bool serialFlag = false;
bool done = false;
// end of global variables

// ISRs
void front_button_isr();
void back_button_isr();
void left_button_isr();
void right_button_isr();
void stop_movement_isr();
void echo_isr();
void ultrasonic_isr();
void safe_isr(); // meaning there is no obstacle
void serial_isr();
// end of ISRs

// helper functions
void move(float m1, float m2, float m3, float m4);
// end of helper functions

// push buttons (microphone sensors)
InterruptIn front_button(p5); 
InterruptIn back_button(p6); 
InterruptIn left_button(p7); 
InterruptIn right_button(p8); 
// end push buttons (microphone sensors)

// tickers
Ticker movement_ticker;
Ticker ultrasonic_ticker;
Ticker only_backwards_ticker;
Ticker safe_ticker;
// end of tickers

// ultrasonic
Ultrasonic* ultrasonic = new Ultrasonic(p19, echo_isr);
// ultrasonic end

// bluetooth switch
DigitalIn bluetoothSwitch(p20);
// end of bluetooth switch

// serial 
Serial pc(USBRX,USBTX);
// end of serials

// move() function is not implemented as a safe function (although it 
// is good practice) because all of the other tasks that use this function
// also check for conditions that involve shared variables. 
void move(float m1, float m2, float m3, float m4){
  in1 = m1;
  in2 = m2;
  in3 = m3;
  in4 = m4;
  movement_ticker.attach(callback(&stop_movement_isr), MOVEMENT_DURATION_S); 
}

void checkMovement(char c){
  CriticalSectionLock::enable();
  switch(c){
      case 'w':
        if(only_backwards == 0){
          move(0.0, speed, 0.0, speed);
        }
        break;
      case 'a':
        move(0,speed,speed,0);
        break;
      case 'd':
        move(speed,0,0,speed);
        break;
      case 's':
        move(speed,0,speed,0);
        break;
      case '1':
        speed = SPEED_1;
        break;
      case '2':
        speed = SPEED_2;
        break;
      default:
        printf("invalid input !\n");
        break;
  CriticalSectionLock::disable();
  }
}

void serial_isr(){
  CriticalSectionLock::enable();
  serialFlag = true;
  CriticalSectionLock::disable();
}

void ultrasonic_isr(){
  CriticalSectionLock::enable();
  done = false;
  safe_ticker.attach(callback(&safe_isr), SAFE_S);
  ultrasonic->trigger();
  CriticalSectionLock::disable();
}

void echo_isr(){
  CriticalSectionLock::enable();
  if(!done){
    done = true;
    only_backwards = 1;
   
    if(in2 != 0.0 && in4 != 0.0){
      in1 = 0;
      in2 = 0;
      in3 = 0;
      in4 = 0;  
    }
  }
  
  safe_ticker.detach();
  CriticalSectionLock::disable();
}

void safe_isr(){
  CriticalSectionLock::enable();
  if(!done){
    done = true;
    only_backwards = 0;
  }
  
  safe_ticker.detach();
  CriticalSectionLock::disable();
}

void stop_movement_isr(){
  CriticalSectionLock::enable();
  in1 = 0;
  in2 = 0;
  in3 = 0;
  in4 = 0;  
  movement_ticker.detach();
  CriticalSectionLock::disable();
}

void front_button_isr() {
  CriticalSectionLock::enable();
  if(only_backwards == 0 && bluetoothSwitch == 0){
    move(0.0, speed, 0.0, speed);
  }
  CriticalSectionLock::disable();
}

void back_button_isr() {
  CriticalSectionLock::enable();
  if(bluetoothSwitch == 0){
    move(speed, 0.0, speed, 0.0);
  }
  CriticalSectionLock::disable();
}

void left_button_isr() {
  CriticalSectionLock::enable();
  if(only_backwards == 0 && bluetoothSwitch == 0){
    move(speed, 0.0, 0.0,speed);
  }
  CriticalSectionLock::disable();
}

void right_button_isr() {
  CriticalSectionLock::enable();
  if(only_backwards == 0 && bluetoothSwitch == 0){
    move(0.0, speed, speed, 0.0);
  }
  CriticalSectionLock::disable();
}


// USER GLOBAL CODE SPACE END

int main() {
  // DO NOT REMOVE THIS CALL FROM MAIN!
  robot_emulator_init();

  // USER MAIN CODE SPACE BEGIN-
  pc.attach(serial_isr);
  speed = SPEED_1;
  
  // tickers
  ultrasonic_ticker.attach(callback(&ultrasonic_isr), ULTRASONIC_PERIOD_S); 

  // buttons
  front_button.fall(callback(&front_button_isr));
  back_button.fall(callback(&back_button_isr));
  left_button.fall(callback(&left_button_isr));
  right_button.fall(callback(&right_button_isr));
  // USER MAIN CODE SPACE END

  while (1) {
    if(bluetoothSwitch == 0 && pc.readable()){
      while (pc.readable()){
        char c = pc.getc();
      }
    }
    
    CriticalSectionLock::enable();
    bool serialFlagTmp = serialFlag;
    CriticalSectionLock::disable();
    
    if(bluetoothSwitch == 1 && serialFlagTmp){
      char c = pc.getc();
      checkMovement(c);
      
      CriticalSectionLock::enable();
      serialFlag = false;
      CriticalSectionLock::disable();
    }
    splitted_wait_ms(10);
  }
}