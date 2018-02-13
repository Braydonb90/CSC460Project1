#include "scheduler.h"

#include <LiquidCrystal.h>
#include <Arduino.h>
#include "Roomba_Driver.h"

#define TEST 1

Roomba r(2, 30);
int LaserPin = 40;

char rhoomba_command = 's';

int posx, posy;
int tilt_max, tilt_min, pan_min, pan_max;
int val_X=512, val_Y=512, val_Z=0;

bool initialized = false;
boolean laser_state = false;

void setup() {  
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
  OCR3B = 250; //250000 / 375 =  666.6Hz  --> 1.5 ms
  OCR3C = 375;


  Serial.begin(9600);
  Serial1.begin(9600);
  if(TEST){
    //posx=(pan_max-pan_min)/2;
    //posy=(tilt_max-tilt_min)/2;
    //update_pan_tilt();
    test_servo();
  }
  else{
    Scheduler_Init();

    //Task args are: delay, period, callback
    Scheduler_StartTask(0, 200, receive_data);
    Scheduler_StartTask(10, 50, update_pan_tilt);
    Scheduler_StartTask(20, 50, update_laser);
    //Scheduler_StartTask(30, 50, control_rhoomba);
    Scheduler_StartTask(40, 50, send_data);
  }
}

void loop() {
  if(TEST){
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

/*-------Rhoomba Stuff--------*/
void control_rhoomba() {
  if(!initialized) {
      r.init();
      initialized = true;
    }

  switch(rhoomba_command)
    {
      case 'f': 
        r.drive(150, 32768);
        break;
      case 'b':
        r.drive(-150, 32768);
        break;
      case 'r':
        r.drive(50, -1);
        break;
      case 'l':
        r.drive(50, 1);
        break;
      case 's':
        r.drive(0,0);
        break;
      case 'd':
        r.dock();
        break;
      case 'p':
        r.power_off();
        initialized = false;
        break;
      default:
        break;
    }
  }



/*-------Laser Stuff---------*/

void update_laser(){
  if(val_Z){//button pressed and released
    val_Z = 0;
      laser_state = !laser_state;
      if(laser_state){
        digitalWrite(LaserPin, HIGH);
      }
      else{
        digitalWrite(LaserPin, LOW);
      }
    }
}
void send_data(){
  char pt[6];
  pt[0] = OCR3B/100;
  pt[1] = (OCR3B % 100) / 10;
  pt[2] = (OCR3B % 10);
  pt[3] = (OCR3C/100);
  pt[4] = (OCR3C % 100) / 10;
  pt[5] = (OCR3C % 10);

  Serial1.write(pt, 6);
}

void receive_data(){
  if(Serial1.available() > 0){
    Serial.println(Serial1.available(), DEC);
    char vals[6];
    Serial1.readBytes(vals, 6);
    int i;
    Serial.print("Incoming: ");
  for(i = 0; i < 3; i++) {
    Serial.print(vals[i], DEC);
  }
  Serial.print(",");
  for(; i < 6; i++) {
    Serial.print(vals[i], DEC);
  }
  Serial.print(",");
  
  Serial.println(vals[6], DEC);
  //Serial.print(",");
  
  //Serial.println(vals[7]);
    
    val_X = vals[0] * 100 + vals[1] * 10 + vals[2];
    val_Y = vals[3] * 100 + vals[4] * 10 + vals[5];
    val_Z = vals[6];

    Serial.print(val_X, DEC);
    Serial.print(",");
    Serial.print(val_Y, DEC);
    Serial.print(",");
    Serial.println(val_Z, DEC);
   // Serial.print(",");

    //rhoomba_command = vals[7];
    //Serial.println(rhoomba_command);
  }
}
/*-----------Servo Stuff ---------*/

void update_pan_tilt(){
  servo_set_pan(val_X);
  servo_set_tilt(val_Y);
}
//Pin2
void servo_set_pan(uint16_t pos) {
  int pan = map(pos, 0, 1023, pan_min, pan_max);
  OCR3B = constrain(pan, pan_min, pan_max);
}

//Pin3
void servo_set_tilt(uint16_t pos) {
  int tilt = map(pos, 0, 1023, tilt_min, tilt_max);
  OCR3C = constrain(tilt, tilt_min, tilt_max);
}



/*----------Testing Stuff----------*/
void test_servo(){
    int pos;
    float period = 5000;
    float t_step = period / 180;
    Serial.println("--- Beginning servo test ---");
    for (pos = 0; pos <= 180; pos++) {
      posy = map(pos, 0, 180, tilt_min, tilt_max);
      posx = map(pos, 0, 180, pan_min, pan_max);
      servo_set_tilt(posy);  
      servo_set_pan(posx); 
      delay(t_step);   
    }
    for (pos = 180; pos > 0; pos--) { 
      posy = map(pos, 0, 180, tilt_min, tilt_max);
      posx = map(pos, 0, 180, pan_min, pan_max);
      servo_set_tilt(posy);  
      servo_set_pan(posx);
      delay(t_step);  
    }
    //back to center
    for (pos = 0; pos < 90; pos++){
      posy = map(pos, 0, 180, tilt_min, tilt_max);
      posx = map(pos, 0, 180, pan_min, pan_max);
      servo_set_tilt(posy);  
      servo_set_pan(posx);     
      delay(t_step);
    }
    Serial.println("--- Servo test finished ---");
}

int time_function(void (*func)(void)){
  int start_t = micros();
  func();
  return micros() - start_t;
}
