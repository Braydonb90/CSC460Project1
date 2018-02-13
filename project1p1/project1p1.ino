#include <LiquidCrystal.h>

#define TEST 0
#define MAX_DIFF 5
#define LIGHT_THRESH 50

int JoyStick_X = A1; // x
int JoyStick_Y = A2; // y
int JoyStick_Z = 42; // button
int LightSensor = A3;
int LaserPin = 40;
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int previous;
int lx, ly, ll;
float alpha = 0.02, light_alpha = 0.02;
int posx,posy, light_value;
int postprint_elapsed = 0, screen_period = 250;
int tilt_max, tilt_min, pan_min, pan_max;
int room_light_level = 0;
boolean laser_state = false, laser_detected = true;
boolean pressed = false;
boolean screen_update_required = false;
void setup() {

  pinMode (JoyStick_X, INPUT);
  pinMode (JoyStick_Y, INPUT);
  pinMode (JoyStick_Z, INPUT_PULLUP);
  pinMode (LightSensor, INPUT);
  pinMode (LaserPin, OUTPUT);
  
  DDRE |= (1<<PE4); //Pin 2 (OC3B/PWM) as OUT
  DDRE |= (1<<PE5); //Pin 3 (OC3C/PWM) as OUT

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
  OCR3B = 375; //250000 / 375 =  666.6Hz  --> 1.5 ms
  OCR3C = 375;

  lx = 512;
  ly = 512;
  posx = 512;
  posy = 512;
  Serial.begin(9600);
  lcd.begin(16,2);
   if(TEST){
      test_servo();
   }
   sample_room_light();
}

void loop() {
    update_joystick();
    update_light_sensor();
    update_lcd(20);
    delay(20);
}
void update_light_sensor(){
    int value = analogRead(LightSensor);
    Serial.println(value, DEC);
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
void update_joystick(){
    int x = joystick_X();
    int y = joystick_Y();
    int z = digitalRead(JoyStick_Z);

    if(!z){     //button pressed
      pressed = true;
    }
    else if(z && pressed){//button released
      pressed = false;
      laser_state = !laser_state;
      if(laser_state){
        digitalWrite(LaserPin, HIGH);
        Serial.println("laser on");
      }
      else{
        digitalWrite(LaserPin, LOW);
        Serial.println("laser off");
      }
    }
    int prev_posx = posx;
    if(x < 512) {
      posx = map(x, 0, 512, posx-MAX_DIFF, posx);
    } else {
      posx = map(x, 512, 1023, posx, posx+MAX_DIFF);
    }    
    int prev_posy = posy;
    if(y < 512) {
      posy = map(y, 0, 512, posy-MAX_DIFF, posy);
    } else {
      posy = map(y, 512, 1023, posy, posy+MAX_DIFF);
    }

    posx = constrain(posx, 0, 1023);
    int pan = map(posx, 0, 1023, pan_min, pan_max);
    servo_set_pan(pan);

    posy = constrain(posy, 0, 1023);
    int tilt = map(posy, 0, 1023, tilt_min, tilt_max);
    servo_set_tilt(tilt);
    
    if(prev_posx != posx || prev_posy != posy)
      screen_update_required = true;
}
void sample_room_light(){
  int sum = 0;
  delay(1000);
  for(int i=0; i<20; i++){
    int val = analogRead(LightSensor);
    sum += val;
    delay(50);
  }
  room_light_level = sum / 20;
  Serial.print("Avg light level for room: ");
  Serial.println(room_light_level);
}
void test_servo(){
    int pos;
    float period = 5000;
    float t_step = period / 180;
    Serial.println("--- Beginning servo test ---");
    lcd.clear();
    for (pos = 0; pos <= 180; pos++) {
      posy = map(pos, 0, 180, tilt_min, tilt_max);
      posx = map(pos, 0, 180, pan_min, pan_max);
      servo_set_tilt(posy);  
      servo_set_pan(posx); 
      delay(t_step);   
      update_lcd(t_step);                              
    }
    for (pos = 180; pos > 0; pos--) { 
      posy = map(pos, 0, 180, tilt_min, tilt_max);
      posx = map(pos, 0, 180, pan_min, pan_max);
      servo_set_tilt(posy);  
      servo_set_pan(posx);     
      delay(t_step);  
      update_lcd(t_step);
    }
    //back to center
    for (pos = 0; pos < 90; pos++){
      posy = map(pos, 0, 180, tilt_min, tilt_max);
      posx = map(pos, 0, 180, pan_min, pan_max);
      servo_set_tilt(posy);  
      servo_set_pan(posx);     
      delay(t_step);  
      update_lcd(t_step);
    }
    Serial.println("--- Servo test finished ---");
}
void update_lcd(int dt){
  if(screen_update_required && postprint_elapsed > screen_period){
        postprint_elapsed = 0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("x: "); //0-2 + 3-6
        lcd.setCursor(2,0);
        lcd.print(posx);
        lcd.setCursor(8,0);
        lcd.print("y: "); //8-10 + 11-14
        lcd.setCursor(11,0);
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
  else{
    postprint_elapsed += dt;
  }
}
int joystick_X() {
  int val_X = analogRead (JoyStick_X);
  int ret_X;
  
  val_X += 8; // offset of 8
  //deadzone range values should still be fed into smoothing filter?
  //if not, deadzone replacement vals should
  lx = (1-alpha)*(float)val_X + alpha*(float)lx;
  
  if(val_X <= 612 && val_X >= 412) {
    ret_X = 512;
  }
  else if(val_X <= 12) {
    ret_X = 0;
  }
  else if(val_X >= 1011) {
    ret_X = 1023;
  }
  else{
    ret_X = lx;  
  }

  return ret_X;
}

int joystick_Y() {
  int val_Y = analogRead (JoyStick_Y);
  int ret_Y;
  
  val_Y += -11; // offset of -11
  ly = (1-alpha)*(float)val_Y + alpha*(float)ly;
  
  if(val_Y <= 612 && val_Y >= 412) {
    ret_Y = 512;
  }
  else if(val_Y <= 12) {
    ret_Y = 0;
  }
  else if(val_Y >= 1011) {
    ret_Y = 1023;
  }
  else{
    ret_Y = ly;  
  }

  return ret_Y;
}

//Pin2
void servo_set_pan(uint16_t pos) {
  OCR3B = constrain(pos, pan_min, pan_max);
}

//Pin3
void servo_set_tilt(uint16_t pos) {
  OCR3C = constrain(pos, tilt_min, tilt_max);
}

