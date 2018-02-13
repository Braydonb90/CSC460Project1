#include "scheduler.h"

#include <LiquidCrystal.h>
#include <Arduino.h>

#define TEST 0
#define MAX_DIFF 2
#define LIGHT_THRESH 50
#define NUM_JOYSTICKS 1
#define DEADZONE 100

typedef struct{
  int pinx;
  int piny;
  int pinz;
  int lx;
  int ly;
  int valx;
  int valy;
  int valz;
  boolean pressed;
}JoyStick;

int LightSensor = A5;
int LaserPin = 40;
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int ll;
float alpha = 0.02, light_alpha = 0.02;
int posx,posy, light_value;
int prev_x=0, prev_y=0;
int screen_period = 250;
int tilt_max, tilt_min, pan_min, pan_max;
int max_joy=0, max_lcd=0, max_ls=0;

JoyStick joysticks[NUM_JOYSTICKS];

int room_light_level = 0;
boolean laser_detected = true;
boolean screen_update_required = false;


void setup() {
  joysticks[0].pinx = A1;
  joysticks[0].piny = A2;
  joysticks[0].pinz = 42;
  joysticks[0].pressed = false;
  joysticks[0].lx = 512;
  joysticks[0].ly = 512;

  
  joysticks[1].pinx = A3;
  joysticks[1].piny = A4;
  joysticks[1].pinz = 44;
  joysticks[1].pressed = false;
  joysticks[1].lx = 512;
  joysticks[1].ly = 512;

  pinMode (joysticks[0].pinx, INPUT);
  pinMode (joysticks[0].piny, INPUT);
  pinMode (joysticks[0].pinz, INPUT_PULLUP);
  pinMode (joysticks[1].pinx, INPUT);
  pinMode (joysticks[1].piny, INPUT);
  pinMode (joysticks[1].pinz, INPUT_PULLUP);

  pinMode (LightSensor, INPUT);
  pinMode (LaserPin, OUTPUT);
  
  DDRE |= (1<<PE4); //Pin 2 (OC3B/PWM) as OUT for pan
  DDRE |= (1<<PE5); //Pin 3 (OC3C/PWM) as OUT for tilt

  //Set timer3 to fast PWM mode
  //This means that counter counts from BOTTOM to TOP. Once top is reached, 
  //OCR registers are updated and TCNT is reset to BOTTOM
  //BOTTOM is 0
  //TOP is whatever OCR3A is set to  
  TCCR3A |= (1<<WGM30) | (1<<WGM31);
  TCCR3B |= (1<<WGM32) | (1<<WGM33);

  TCCR3A |= (1<<COM3C1); //Clear OCR3C on compare match. Set OCR3C at bottom
  TCCR3A |= (1<<COM3B1); //Clear OCR3B on compare match. Set OCR3B at bottom

  //Clock select 3: Clock_IO / 64 = 250kHz
  TCCR3B |= (1<<CS31)|(1<<CS30);

  tilt_min = 250;
  tilt_max = 620;
  pan_min = 250;
  pan_max = 550;
  
  //PWM 
  //Manual Page 147
  //OCR match when OCR val == CNT
  //The value of OCR3A is used as the period
  //Since COM3C1 and COM3B1 are set, each period starts with OCR3C/OCR3B's corresponding pins being high
  //Once OCR3C/OCR3B matches TCNT, the corresponding pin is set to low.
  OCR3A = 5000; //250000 / 5000 = 50Hz  --> 20 ms period
  OCR3B = 350; //250000 / 375 =  666.6Hz  --> 1.5 ms
  OCR3C = 714;

  posx=(pan_max-pan_min)/2;
  posy=(tilt_max-tilt_min)/2;
  query_joysticks();
  update_pan_tilt();
  //posx = (pan_max - pan_min) / 2;
  //posy = (tilt_max - tilt_min) / 2;
  //servo_set_tilt(posy);
  //servo_set_pan(posx);
  Serial.begin(9600);
  Serial1.begin(9600);
  lcd.begin(16,2);
  sample_room_light();
  if(TEST){
  }
  else{
    Scheduler_Init();

    //Task args are: delay, period, callback
    Scheduler_StartTask(0, 100, query_joysticks);
    Scheduler_StartTask(10, 50, receive_data);
    Scheduler_StartTask(20, 50, update_pan_tilt);
    Scheduler_StartTask(30, 200, send_data);
    Scheduler_StartTask(50, 300, query_light_sensor);
    Scheduler_StartTask(100, 250, update_lcd);
  }
}

void loop() {
  if(TEST){
    test_io_times();
  }
  else{
    run_scheduler();
  }    
}

/*-------Scheduler Stuff-----------*/

void run_scheduler(){
  uint32_t idle_period = Scheduler_Dispatch();
  if(idle_period){
    delay(idle_period);
  }
}


/*-------Laser/LightSensor Stuff---------*/

void query_light_sensor(){
    int value = analogRead(LightSensor);
    ll = (1-alpha)*(float)value + alpha*(float)ll;
    boolean prev_laser_detected = laser_detected;
    if(ll > room_light_level + LIGHT_THRESH){
      laser_detected = true; 
    }
    else{
      laser_detected = false;
    }
    if(laser_detected != prev_laser_detected){
      screen_update_required = true;
    }
}
void sample_room_light(){
  int sum = 0;
  delay(500);
  lcd.setCursor(0,0);
  lcd.print("SAMPLING LIGHT");
  for(int i=0; i<20; i++){
    int val = analogRead(LightSensor);
    sum += val;
    update_pan_tilt();
    delay(50);
  }
  room_light_level = sum / 20;
  Serial.print("Avg light level for room: ");
  Serial.println(room_light_level);
  lcd.clear();
}

void receive_data(){
  digitalWrite(LaserPin, HIGH);
  if(Serial.available() > 0){
    char vals[6];
    Serial.readBytes(vals, 6);

  }
  digitalWrite(LaserPin, LOW);
}

void send_data(){
  char pt[7];
  
  pt[0] = posx/100;
  pt[1] = (posx % 100) / 10;
  pt[2] = (posx % 10);
  pt[3] = (posy/100);
  pt[4] = (posy % 100) / 10;
  pt[5] = (posy % 10);

  if(joysticks[0].valz && joysticks[0].pressed){//button pressed and released
      joysticks[0].pressed = false;
      pt[6] = 1;
    } else {
      pt[6] = 0;
    }

  
  pt[7] = joysticks[1].valx > 562 ? 'r' : (joysticks[1].valx < 462 ? 'l' : 's');

  pt[7] = joysticks[1].valy > 562 ? 'f' : (joysticks[1].valy < 462 ? 'b' : pt[7]);
  
  if(!joysticks[1].valz) {
    if(pt[7] == 's') {
      pt[7] = 'p';
    } else {
      pt[7] = 'd';
    }
  }

char c = joysticks[1].valx > 562 ? 'r' : (joysticks[1].valx < 462 ? 'l' : 's');

  c = joysticks[1].valy > 562 ? 'f' : (joysticks[1].valy < 462 ? 'b' : c);
  
  if(!joysticks[1].valz) {
    if(c == 's') {
      c = 'p';
    } else {
      c = 'd';
    }
  }
  Serial.println(c);

  Serial1.write(c);
}

/*------------LCD Stuff-------------*/

//TODO: Should really only change parts of the display (numbers and ls value)
void update_lcd(){
  if(screen_update_required){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("p: "); //0-2 + 3-6
        lcd.print(posx);
        lcd.setCursor(8,0);
        lcd.print("t: "); //8-10 + 11-14
        lcd.print(posy);

        lcd.setCursor(0,1);
        lcd.print("laser: ");
        if(laser_detected){
          lcd.print("on");
        }
        else{
          lcd.print("off");
        }
        screen_update_required = false;   
  }
}


/*--------JoyStick Stuff-----------*/

void query_joysticks(){
  for(int i=0; i<NUM_JOYSTICKS; i++){
    joysticks[i].valx = joystick_X(i);
    joysticks[i].valy = joystick_Y(i);
    joysticks[i].valz = digitalRead(joysticks[i].pinz);
  
    if(!joysticks[i].valz){     //button pressed
      joysticks[i].pressed = true;
    }
  }
  //Serial.print(joysticks[0].valx);
  //Serial.print(" ");
  //Serial.println(joysticks[0].valy);
  if(prev_x != joysticks[0].valx || prev_y != joysticks[0].valy)
  {
    prev_x = joysticks[0].valx;
    prev_y = joysticks[0].valy;
    screen_update_required = true; 
  } 
  
}
int joystick_X(int i) {
  int val_X = analogRead (joysticks[i].pinx);
  int ret_X;

  //Serial.print("x read as: ");
  //Serial.println(val_X);
  //deadzone range values should still be fed into smoothing filter?
  //if not, deadzone replacement vals should
  joysticks[i].lx = (1-alpha)*(float)val_X + alpha*(float)joysticks[i].lx;
  
  if(val_X <= (512+DEADZONE) && val_X >= (512-DEADZONE)) {
    ret_X = 512;
  }
  else if(val_X <= 12) {
    ret_X = 0;
  }
  else if(val_X >= 1011) {
    ret_X = 1023;
  }
  else{
    ret_X = joysticks[i].lx;  
  }

  return ret_X;
}

int joystick_Y(int i) {
  int val_Y = analogRead (joysticks[i].piny);
  //Serial.print("y read as: ");
  //Serial.println(val_Y);
  //Serial.println();
  int ret_Y;
  joysticks[i].ly = (1-alpha)*(float)val_Y + alpha*(float)joysticks[i].ly;
  
  if(val_Y <= (512+DEADZONE) && val_Y >= (512-DEADZONE)) {
    ret_Y = 512;
  }
  else if(val_Y <= 12) {
    ret_Y = 0;
  }
  else if(val_Y >= 1011) {
    ret_Y = 1023;
  }
  else{
    ret_Y = joysticks[i].ly;  
  }

  return ret_Y;
}

/*-----------Servo Stuff ---------*/

//definitely some extra constrain/maps atm
void update_pan_tilt(){
  int prev_posx = posx;
  if(joysticks[0].valx < 512) {
    posx = map(joysticks[0].valx, 0, 512, posx-MAX_DIFF, posx);
  } else {
    posx = map(joysticks[0].valx, 512, 1023, posx, posx+MAX_DIFF);
  }    
  int prev_posy = posy;
  if(joysticks[0].valy < 512) {
    posy = map(joysticks[0].valy, 0, 512, posy-MAX_DIFF, posy);
  } else {
    posy = map(joysticks[0].valy, 512, 1023, posy, posy+MAX_DIFF);
  }

  posx = constrain(posx, 0, 1023);
  posy = constrain(posy, 0, 1023);  
  if(prev_posx != posx || prev_posy != posy)
    screen_update_required = true;
}



/*----------Testing Stuff----------*/

void test_io_times(){
  int joy_t = time_function(query_joysticks);
  int ls_t = time_function(query_light_sensor);
  int lcd_t = time_function(update_lcd);
  if(joy_t > max_joy){
    max_joy = joy_t;
    Serial.print("new joy max: ");
    Serial.println(max_joy);
  }
  if(ls_t > max_ls){
    max_ls = ls_t;
    Serial.print("new ls max: ");
    Serial.println(max_ls);
  }
  if(lcd_t > max_lcd){
    max_lcd = lcd_t;
    Serial.print("new lcd max: ");
    Serial.println(max_lcd);
  }
}
int time_function(void (*func)(void)){
  int start_t = micros();
  func();
  return micros() - start_t;
}
