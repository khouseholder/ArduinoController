/****************************************************
 * this project was created 4-10-17 
 * For: PAAL controller 
 * Rev: 0.0
 ****************************************************/
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <stdint.h>
#include "TouchScreen.h"
#include <Adafruit_ILI9341.h>

/****************************************************
 * Comment out the below line in order to remove Debug messages
****************************************************/
//#define DEBUG

/****************************************************
 * Comment out the below line in order to turn off the direct rdac control
****************************************************/
//#define FLOSSY

/****************************************************
 * Comment out the below line in order to turn off eemem
****************************************************/
//#define POT_EEMEM

#define RPM_TIMEOUT 2000
#define SET_SPEED_MIN 500
#define POT_TIME_THRESH 10 //this is the minimum time in ms for changing the motorspeed
#define POT_TIME_MAX 1000
#define POT_DECEL_THRESH 70
#define POT_BAUD_RATE 2000000
#define POT_SPI_MODE SPI_MODE0
#define RPM_VOLTAGE_THRESH 400

#define YP A8  // screen pin y+
#define XM A9  // screen pin x-
#define YM 31   // screen pin y-
#define XP 29   // screen pin x+

#define BLACK   0x0000
#define RED     0xF800
#define GREEN   0x07E0
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define BUTTON_HEIGHT 80
#define BUTTON_RADIUS 20
#define RPM_TEXT_HEIGHT 110 //kthe y offset of the rpm text
#define RPM_TEXT_OFFSET 100
#define RPM_CHAR_OFFSET 53  //spacing between cursors for writing rpm

#define RPM_DIG_0 RPM_TEXT_OFFSET
#define RPM_DIG_1 RPM_DIG_0+RPM_CHAR_OFFSET
#define RPM_DIG_2 RPM_DIG_1+RPM_CHAR_OFFSET
#define RPM_DIG_3 RPM_DIG_2+RPM_CHAR_OFFSET
#define RPM_DIG_4 RPM_DIG_5+RPM_CHAR_OFFSET
#define RPM_DIG_5 RPM_DIG_6+RPM_CHAR_OFFSET

#define TFT_DC_PIN 27
#define TFT_CS_PIN 25 

#define RPM_ANALOG_PIN 0
//definitions of the analog pins connected to the individual sensor inputs: 5/22/2017
#define SENSOR1_ANALOG_PIN 1
#define SENSOR2_ANALOG_PIN 2
#define SENSOR3_ANALOG_PIN 3
#define SENSOR4_ANALOG_PIN 4

#define POT_CS_PIN 23
#define POT_RDY_PIN 33

#define SPEED_UPPER_BOUND 2500

#define SENSOR_READ_COMMAND 's'
#define LABVIEW_PACKET_ENDING '\n'

#define SCREEN 0


/****************************************************
 * Function prototypes
****************************************************/
char check_for_commands(void);
char check_for_screen(void);
int parse_speed(void);
void set_speed(int);
void rdac_to_pot(int);
void to_pot(int);
short from_pot(void);
void print_data(void);
void update_rpm(void);
void reset_rpm_measurment(void);
void reset_display(void);
void display_rpm(void);
char check_rdac_stability(void);

#ifdef SCREEN
int is_screen_touched(void);
int is_exit_button_pressed(void);
int is_set_button_pressed(void);
int one_button_pressed(void);
int two_button_pressed(void);
int three_button_pressed(void);
int four_button_pressed(void);
int five_button_pressed(void);
int six_button_pressed(void);
int seven_button_pressed(void);
int eight_button_pressed(void);
int nine_button_pressed(void);
int zero_button_pressed(void);
int back_button_pressed(void);
void print_interactive_screen(void);
void print_int_set_speed(void);
#endif

/****************************************************
 * global variables and messageers
****************************************************/
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS_PIN, TFT_DC_PIN);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint p;

float diff,last_diff,rpm=0;
float last_rpm=0;

int speed_request=0;
int i=0;

int rpm_a_in;
char temp_char,message[100];
//messaging message;
char message_end=0;
unsigned char hall_ticks=0;
char is_high=0,is_low=1;
unsigned long start_time,end_time,last_rdac,last_rpm_time;

char screen_flag=0;
char rpm_reset_flag=0;
int rpm_zed_timer=0;
int rdac=0;
unsigned long pot_time=POT_TIME_THRESH;
char stable_rdac_flag=0;
float curr_rpm_diff=0,last_rpm_diff=0;

#ifdef SCREEN
int screen_mode = 0; // 0 for display screen, 1 for interaction screen
int interactive_set_speed = 0; // will be modified via interaction with screen
                               // via the up and down buttons
int buttons_active = 1; // 0 when buttons are inactive
                        // 1 when buttons are active
#endif

/****************************************************
 * setup code
 * This code does the following things in a particular order:
 * Set up spi
 * Read last set_speed
 * if(last_speed!=0)
 * *Measure abridged rpm
 * *Guess pot setting
 * *set_speed 
 * screen init
 * 
 * pin assignments
 * *
****************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.setTimeout(10);
  Serial.begin(115200);

  //SPI.setClockDivider(SPI_CLOCK_DIV2);  
  SPI.begin();
  pinMode(POT_CS_PIN, OUTPUT);
  pinMode(POT_RDY_PIN,INPUT);
  pinMode(13,OUTPUT);
  digitalWrite(POT_CS_PIN,HIGH);

  reset_display();

  #ifdef DEBUG
  Serial.println("dbg msg: setup done");
  #endif
  rdac_to_pot(rdac);
}

/****************************************************
 * The main loop is summed up by the following psuedo code:
  
 * if(Serial_Data_available){
 *  do_set_speed_or_send_data();
 * }else if(Screen_senses_activity){
 *  Code to handle screen
 * }else{
 *  set the pot to seek speed
 *  measure the rpm
 * }
 
 * 
****************************************************/
//unsigned char bytes;
int inc,rdac_index=0,stab_refresh_counter=0;
long last_stab_milli;
int past_rdac[10];
float past_diffs[10];
char stab_counter;

void loop() {

  #ifdef SCREEN
    p = ts.getPoint();
  #endif
 
 if(check_for_commands()){
  rpm_reset_flag=1;
  //code to parse the command then carry it out
  while(Serial.available()){ 
     temp_char=Serial.read();
    message[i++]=temp_char;
   }  
  message_end=i-1;
  if(message[message_end]=='\n')
  {
    i=0;
    #ifdef DEBUG
    Serial.print("dbg msg: ");
    Serial.print("message rcved:");
    message[message_end]=0;
    Serial.println(message);
    message[message_end]='\n';    
    #endif
    if(message[0]==SENSOR_READ_COMMAND){
      //print data and check that there isn't a set speed command
     #ifdef DEBUG
     Serial.print("dbg msg: ");
     Serial.println("sensor read");
      #endif
      print_data();
    }else{
     set_speed(parse_speed());
    }

    //resetting the message buffer
    for(inc=0;inc<message_end+3;inc++){
      message[inc]=0;
    }
  }else{
    
  }
 }else if(check_for_screen()){
  rpm_reset_flag=1;
  //put all screen related code here
  #ifdef SCREEN
    if(screen_mode == 1)
    {
      print_int_set_speed();
    }
    //put all screen related code here
    if(is_screen_touched() == 1 && screen_mode == 0 && buttons_active == 1)
    {
      buttons_active = 0;
      screen_mode = 1;
      print_interactive_screen();
    }
    else if(is_exit_button_pressed() == 1 && buttons_active == 1)
    {
      buttons_active = 0;
      screen_mode = 0;
      interactive_set_speed = 0;
      reset_display();
    }
    else if(is_set_button_pressed() == 1 && buttons_active == 1)
    {
      buttons_active = 0;
      screen_mode = 0;

      //put sanity checks on interactive set speed
      if(interactive_set_speed>=300 && interactive_set_speed <2500){
        set_speed(interactive_set_speed);
      }else if(interactive_set_speed<300){
        set_speed(0);
      }else{
        set_speed(speed_request);
      }
      
      interactive_set_speed = 0;
    }
    else if(interactive_set_speed <= 250 && buttons_active == 1 && screen_mode == 1) // to make sure we don't allow for ridiculous requested speeds
    {
      if(one_button_pressed()== 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 1;
      }
      else if(two_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 2;
      }
      else if(three_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 3;
      }
      else if(four_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 4;
      }
      else if(five_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 5;
      }
      else if(six_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 6;
      }
      else if(seven_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 7;
      }
      else if(eight_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 8;
      }
      else if(nine_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10 + 9;
      }
      else if(zero_button_pressed() == 1)
      {
        buttons_active = 0;
        interactive_set_speed = interactive_set_speed * 10;
      }
      print_int_set_speed();
    }
    else
    {
      // No buttons pressed
    }
  #endif
  screen_flag=0;
  
 }else{

  #ifdef SCREEN
    buttons_active = 1; // nothing is being pressed so we can now go to pressing buttons again
  #endif
  //put the rpm measuring code and speed setting code here



//the following conditional is the entire code to set the motor speed
    if((millis()-last_rdac)>pot_time){
      diff=(float)speed_request-rpm;
      if(abs(diff)<10 && pot_time<POT_TIME_MAX*2){
        pot_time=POT_TIME_MAX*2;
      }else if(abs(diff)<50 && pot_time<POT_TIME_MAX/2){
        if(speed_request<700){
        pot_time=POT_TIME_MAX;
        }else{
        pot_time=POT_TIME_MAX/4;  
        }
      }else if(abs(diff)<200  && pot_time<POT_TIME_MAX){
      pot_time=3*POT_TIME_THRESH;
      }else if(abs(diff)>200){
        pot_time=POT_TIME_THRESH;
      }
      if(abs(diff)<3){
      }else if(rpm>(float)speed_request && rdac>0){
         rdac_to_pot(--rdac);
      }else if(rpm<(float)speed_request && rdac<1023){
         rdac_to_pot(++rdac);
      } 
      last_rdac=millis();
    } 

/*
 
 //the following conditional is the entire code to set the motor speed
  if(!stable_rdac_flag){//checking if we need to do anything

    if((millis()-last_rdac)>pot_time){//checking that the timer for the wiper is ready
     last_rdac=millis();
     diff=(float)speed_request-rpm;
     if(diff>100){
      pot_time=POT_TIME_THRESH;
     }
      if(check_rdac_stability() && abs(diff)<STAB_RPM_DIFF){//if we are close to the desired rpm and we have reached stable rdac values
        pot_time=1000;//big wiper delay
        stab_counter++;//stability counter
        #ifdef DEBUG
        Serial.println("dbg msg: rdac nearly stable");
        #endif
        past_diffs[rdac_index%10]=diff;//tracking the last differences
        rdac_to_pot(past_rdac[rdac_index%10]);
        rdac=past_rdac[rdac_index%10];
        if(stab_counter>10){
          stab_counter=0;
          if(past_diffs[0]<past_diffs[1]){
            rdac=past_rdac[0];
          }else{
            rdac=past_rdac[1];
          }
          //rdac_to_pot(rdac);
          stable_rdac_flag=1;
         #ifdef DEBUG
        Serial.println("dbg msg: rdac stable");
        #endif
        }
      }else{
        stab_counter=0;
        if(diff<0 && rdac>0 || !speed_request){
          if(diff>-POT_DECEL_THRESH&&pot_time<POT_TIME_MAX){
            pot_time*=2;
         }
          rdac_to_pot(--rdac);
        }else if(diff>=0 && rdac<1023){
          rdac_to_pot(++rdac);
          if(diff<POT_DECEL_THRESH&&pot_time<POT_TIME_MAX){
           pot_time*=2;
         }
        }else{
          #ifdef DEBUG
          Serial.print("dbg msg: ");
          Serial.println("hopefully noone sees this");
          #endif  
      }
      past_rdac[(++rdac_index)%10]=rdac;
      last_diff=diff;
      if(!rdac && !speed_request){
        stable_rdac_flag=1;
      }else if(speed_request>SET_SPEED_MIN &&rdac==1023){
        stable_rdac_flag=1;
      }
      
    } 
   }
    
  }else{//close if(not stable condition)
    if(!speed_request && !rdac){
      ;
    }else if(last_stab_milli!=millis()){
      stab_refresh_counter++;
      last_stab_milli=millis();
      if(stab_refresh_counter>STAB_REFRESH){
        stable_rdac_flag=0;
        stab_refresh_counter=0;
        pot_time=POT_TIME_THRESH;
        #ifdef DEBUG
        Serial.println("dbg msg: refresh stability");
        #endif
      }
   }        
  }//close motor speed conditional
*/

  //do rpm measuring stuff here

   rpm_a_in=analogRead(RPM_ANALOG_PIN);
   if(rpm_reset_flag){
    rpm_reset_flag=0;
    hall_ticks=0;
    if(rpm_a_in>RPM_VOLTAGE_THRESH){
      is_high=1;
      is_low=0;
    }else{
      is_low=1;
      is_high=0;
    }
  }else if(is_low&&(rpm_a_in>RPM_VOLTAGE_THRESH)){
  //  Serial.println(hall_ticks);
      rpm_zed_timer=0;
      is_high=1;
      is_low=0;
      hall_ticks++;
      if(hall_ticks==1){
        start_time=micros();
      }else if(hall_ticks==25){
        end_time=micros();
        update_rpm();
        rpm_reset_flag=1;
      }
    }else if(is_high&&(rpm_a_in<=RPM_VOLTAGE_THRESH)&&(!is_low)){
     rpm_zed_timer=0;
      is_high=0;
      is_low=1;
    }else{
      if(millis()!=last_rpm_time){
        last_rpm_time=millis();
        rpm_zed_timer++;
        if(rpm_zed_timer>RPM_TIMEOUT){
          rpm=0;
          display_rpm();
        }
      }
    }
 }//end rpm calc
  
}//end loop

char check_for_commands(){
 return Serial.available();
}//end check_for_commands

/****************************************************
****************************************************/
char check_for_screen(){
  if(last_rpm!=rpm){
    screen_flag=1;
    last_rpm=rpm;
  }
  else if(p.z > 50)
  {
    return 1;
  }
  return 0;
}//end check_for_screen

/****************************************************
****************************************************/
void set_speed(int new_speed){
  stable_rdac_flag=0;
  #ifndef FLOSSY
  speed_request=new_speed;
  //writes the speed request to memory
  #ifdef POT_EEMEM
  to_pot(speed_request);
  #endif
  //this defaults wiper speed to max
  pot_time=POT_TIME_THRESH;
  #endif
  #ifdef FLOSSY
  rdac_to_pot(new_speed);
  #ifdef POT_EEMEM
  to_pot(new_speed);
  #endif
  #endif
  #ifdef DEBUG
  Serial.print("dbg msg: set_speed:  ");
  Serial.println(new_speed);
  #endif
  reset_display();
  //rdac_to_pot(speed_request);
}

/****************************************************
 * uses the command messageer to determine if a valid speed has been sent
 * 
****************************************************/
int parse_speed(){
  int temp=speed_request;
  unsigned char index=0,k;
  
  while(message[index]!=LABVIEW_PACKET_ENDING){
    if((message[index]-'0')>9 || (message[index]-'0')<0){
      #ifdef DEBUG
      Serial.print("dbg msg: ");
      Serial.println("invalid speed set");
      #endif
      return temp;
    }
    index++;
  }
  message[index]='\0';
  temp= atoi(message);
  for(k=0;k<index;k++){
    message[k]=0;
  }
  if(temp>SPEED_UPPER_BOUND){
    temp=SPEED_UPPER_BOUND;
  }
  #ifndef FLOSSY
  else if(temp<250){
    #ifdef DEBUG
    Serial.print("dbg msg: ");
    Serial.println("Speed to less than thresh");
    #endif  
    return 0;
  }
  #endif
  return temp;
}

/****************************************************
 * this writes a value between 0 and 1023
 * that value will be the new wiper position
****************************************************/
void rdac_to_pot(int num){
 unsigned char msg[3];
 msg[0]=0xb0;
 msg[1]=(unsigned char)((num>>8)&0x3);
 msg[2]=(unsigned char)(num&0xff);
// data1=data1<<6;
 SPI.beginTransaction(SPISettings(POT_BAUD_RATE,MSBFIRST,POT_SPI_MODE));
 digitalWrite(POT_CS_PIN,LOW);
 SPI.transfer(msg,3);
 digitalWrite(POT_CS_PIN,HIGH);
 SPI.endTransaction();
 digitalWrite(13,!digitalRead(13));

   #ifdef DEBUG
  Serial.print("dbg msg: rdac write ");
  Serial.println(num);
  Serial.print("dbg msg: rpm ");
  Serial.println(rpm);

  
  #endif 
}

/****************************************************
 * this writes the last speed_request to eemem on the potentiometer
****************************************************/
void to_pot(int num){
 unsigned char msg[3];
 msg[0]=0x32;
 msg[1]=(unsigned char)((num>>8)&0xff);
 msg[2]=(unsigned char)(num&0xff);
// data1=data1<<6;
 SPI.beginTransaction(SPISettings(POT_BAUD_RATE,MSBFIRST,POT_SPI_MODE));
 digitalWrite(POT_CS_PIN,LOW);
 SPI.transfer(msg,3);
 digitalWrite(POT_CS_PIN,HIGH);
  SPI.endTransaction();
   while(digitalRead(POT_RDY_PIN)){
    
  }
 // msg[0]=0;
 // msg[1]=0;
 // msg[2]=0;
 //SPI.beginTransaction(SPISettings(POT_BAUD_RATE,MSBFIRST,POT_SPI_MODE));
 //digitalWrite(POT_CS_PIN,LOW);
 //SPI.transfer(msg,3);
// digitalWrite(POT_CS_PIN,HIGH);
 //SPI.endTransaction();
 
 

   #ifdef DEBUG
  Serial.print("dbg msg: eemem write ");
  Serial.println(num);
  #endif 
//  delay(25);
  
}

/****************************************************
 *  function returns the number read from the pot eemem
****************************************************/
short from_pot(){
  short num;
 unsigned char out[3],in[3];
 out[0]=0x92;
 out[1]=0;
 out[2]=0;
// data1=data1<<6;
 SPI.beginTransaction(SPISettings(POT_BAUD_RATE,MSBFIRST,POT_SPI_MODE));
 digitalWrite(POT_CS_PIN,LOW);
 SPI.transfer(out,3);
 digitalWrite(POT_CS_PIN,HIGH);
 SPI.endTransaction();
  while(digitalRead(POT_RDY_PIN)){
  }    
 
 SPI.beginTransaction(SPISettings(POT_BAUD_RATE,MSBFIRST,POT_SPI_MODE));
 digitalWrite(POT_CS_PIN,LOW);
 in[0]=SPI.transfer(0);
 num=SPI.transfer16(0); 
 digitalWrite(POT_CS_PIN,HIGH);
  SPI.endTransaction();
// num=(short)(in[1]<<8 | in[2]);
   #ifdef DEBUG
  Serial.print("dbg msg: eemem read ");
    Serial.println(num);

  Serial.print("dbg msg: eemem chk:  ");
  Serial.println(in[0]);
  #endif 
  return num;
}


/****************************************************
 * this is the function that sends sensor data to labview
****************************************************/
void print_data(){
  //Serial.flush();
  Serial.print(rpm);
  Serial.print(' ');
  Serial.print(((float)analogRead(SENSOR1_ANALOG_PIN))*5.0/1024.0);
  Serial.print(' ');
  Serial.print(((float)analogRead(SENSOR2_ANALOG_PIN))*5.0/1024.0);
  Serial.print(' ');
  Serial.print(((float)analogRead(SENSOR3_ANALOG_PIN))*5.0/1024.0);
  Serial.print(' ');
  Serial.println(((float)analogRead(SENSOR4_ANALOG_PIN))*5.0/1024.0);
  #ifdef POT_EEMEM
  Serial.println(from_pot());
  #endif
}//end print_data

/****************************************************
 * this function does the time conversion to find rpm
****************************************************/
void update_rpm(){
  unsigned long diff=end_time-start_time;
  static char flip;
  last_rpm=rpm;
  rpm=360000000.0/((float)diff);
  if(!screen_mode){
      display_rpm();
  }
  #ifdef DEBUG
// Serial.print("dbg msg: rpm ");
// Serial.println(rpm);
  #endif 
}

/****************************************************
 * this function writes the default screen
****************************************************/
void reset_display(){
  char message[8];
  memset(message,0,8);
  itoa((int)round(rpm), message, 10);
  tft.begin();
  tft.setRotation(-45);
  tft.fillScreen(BLACK);
  //tft.fillRect(0, 0, tft.width(), BUTTON_HEIGHT, GREEN); 
  //ft.fillRect(0, tft.height() - BUTTON_HEIGHT, tft.width(), tft.height(), RED); 
   screen_mode = 0;
  
  tft.setCursor(0,125);
  tft.setTextColor(ILI9341_WHITE);  
  tft.setTextSize(4);
  tft.print("RPM: ");
  tft.setCursor(0,5);
  tft.setTextSize(3);
  tft.print("target RPM: ");
  tft.print(speed_request);
  tft.fillRect(RPM_DIG_0,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
  //display_rpm();
  tft.setTextSize(9);
  tft.fillRect(RPM_DIG_0,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
  tft.setCursor(RPM_DIG_0, RPM_TEXT_HEIGHT);
  tft.print(message[0]);
  tft.fillRect(RPM_DIG_1,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
  tft.setCursor(RPM_DIG_1, RPM_TEXT_HEIGHT);
  tft.print(message[1]);
  tft.fillRect(RPM_DIG_2,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
  tft.setCursor(RPM_DIG_2, RPM_TEXT_HEIGHT);
  tft.print(message[2]);
  tft.fillRect(RPM_DIG_3,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
  tft.setCursor(RPM_DIG_3, RPM_TEXT_HEIGHT);
  tft.print(message[3]);
}

/****************************************************
 * this function writes the current rpm minimially
****************************************************/
void  display_rpm(){
    char message[8];
    static char past_message[8];
    static float last,temp;
    temp=round(rpm);
    if(last!=temp){
     memset(message,0,8);
     tft.setTextSize(9);
     itoa((int)temp, message, 10);
     if(past_message[0]!=message[0]){
      tft.fillRect(RPM_DIG_0,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
      tft.setCursor(RPM_DIG_0, RPM_TEXT_HEIGHT);
      tft.print(message[0]);
     }
     if(past_message[1]!=message[1]){
      tft.fillRect(RPM_DIG_1,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
      tft.setCursor(RPM_DIG_1, RPM_TEXT_HEIGHT);
      tft.print(message[1]);
     }
    if(past_message[2]!=message[2]){
      tft.fillRect(RPM_DIG_2,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
      tft.setCursor(RPM_DIG_2, RPM_TEXT_HEIGHT);
      tft.print(message[2]);
     }
     if(past_message[3]!=message[3]){
      tft.fillRect(RPM_DIG_3,RPM_TEXT_HEIGHT,RPM_CHAR_OFFSET, 70, BLACK); 
      tft.setCursor(RPM_DIG_3, RPM_TEXT_HEIGHT);
      tft.print(message[3]);
     }
      itoa((int)temp, past_message, 10);
    }
    last=temp;
}

int is_screen_touched(void)
{
  if(p.z > 5)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int is_exit_button_pressed(void)
{
    if((p.x > 780 && p.x < 870) && (p.y > 670 && p.y < 900))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int is_set_button_pressed(void)
{
    if((p.x > 780 && p.x < 870) && (p.y > 100 && p.y < 350))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int one_button_pressed(void)
{
  if((p.x > 410 && p.x < 470) && (p.y > 670 && p.y < 900))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int two_button_pressed(void)
{
    if((p.x > 410 && p.x < 470) && (p.y > 370 && p.y < 630))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int three_button_pressed(void)
{
    if((p.x > 410 && p.x < 470) && (p.y > 100 && p.y < 350))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int four_button_pressed(void)
{
    if((p.x > 550 && p.x < 630) && (p.y > 670 && p.y < 900))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int five_button_pressed(void)
{
    if((p.x > 550 && p.x < 630) && (p.y > 370 && p.y < 630))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int six_button_pressed(void)
{
    if((p.x > 550 && p.x < 630) && (p.y > 100 && p.y < 350))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int seven_button_pressed(void)
{
    if((p.x > 670 && p.x < 760) && (p.y > 670 && p.y < 900))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int eight_button_pressed(void)
{
    if((p.x > 670 && p.x < 760) && (p.y > 370 && p.y < 630))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int nine_button_pressed(void)
{
    if((p.x > 670 && p.x < 760) && (p.y > 100 && p.y < 350))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int zero_button_pressed(void)
{
    if((p.x > 780 && p.x < 870) && (p.y > 370 && p.y < 630))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void print_interactive_screen(void)
{
  tft.fillRect(0, 0, tft.width(), tft.height(), BLACK);
  
  print_int_set_speed();
  
  tft.fillRect(0, tft.height() / 3, (tft.width() / 3) - 3,(tft.height() / 6), WHITE);  
  tft.setCursor(tft.width()/6 - 10, tft.height()/3 + tft.height()/12 - 11);
  tft.setTextColor(BLACK);
  tft.setTextSize(4);
  tft.print("1");
  tft.fillRect((tft.width() / 3) + 3, tft.height() / 3, (tft.width() / 3) - 6,(tft.height() / 6), WHITE);
  tft.setCursor(tft.width()/6 - 10 + tft.width() / 3, tft.height()/3 + tft.height()/12 - 11);
  tft.print("2");
  tft.fillRect((2 * tft.width() / 3) + 3, tft.height() / 3, (tft.width() / 3) - 3,(tft.height() / 6), WHITE);
  tft.setCursor(tft.width()/6 - 10 + 2 * + tft.width() / 3, tft.height()/3 + tft.height()/12 - 11);
  tft.print("3");
  tft.fillRect(0, (tft.height() / 3) + (tft.height() / 6) + 3, (tft.width() / 3) - 3,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10, tft.height()/3 + tft.height()/12  + tft.height() / 6 - 11);
  tft.print("4");
  tft.fillRect((tft.width() / 3) + 3, (tft.height() / 3) + (tft.height() / 6) + 3, (tft.width() / 3) - 6,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10 + tft.width() / 3, tft.height()/3 + tft.height()/12 + tft.height() / 6 - 11);
  tft.print("5");
  tft.fillRect((2 * tft.width() / 3) + 3, (tft.height() / 3) + (tft.height() / 6) + 3, (tft.width() / 3) - 3,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10 + 2 * + tft.width() / 3, tft.height()/3 + tft.height()/12  + tft.height() / 6 - 11);
  tft.print("6");
  tft.fillRect(0, (tft.height() / 3) + 2 * (tft.height() / 6) + 3,(tft.width() / 3) - 3, (tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10, tft.height()/3 + tft.height()/12  + 2 * tft.height() / 6 - 11);
  tft.print("7");
  tft.fillRect((tft.width() / 3) + 3, (tft.height() / 3) + 2 * (tft.height() / 6) + 3, (tft.width() / 3) - 6,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10 + tft.width() / 3, tft.height()/3 + tft.height()/12  + 2 * tft.height() / 6 - 11);
  tft.print("8");
  tft.fillRect((2 * tft.width() / 3) + 3,(tft.height() / 3) + 2 * (tft.height() / 6) + 3, (tft.width() / 3) - 3,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10 + 2 * + tft.width() / 3, tft.height()/3 + tft.height()/12  + 2 * tft.height() / 6 - 11);
  tft.print("9");
  tft.fillRect(0, (tft.height() / 3) + 3 * (tft.height() / 6) + 3, (tft.width() / 3) - 3,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 46, tft.height()/3 + tft.height()/12  + 3 * tft.height() / 6 - 11);
  tft.print("Exit");
  tft.fillRect((tft.width() / 3) + 3,(tft.height() / 3) + 3 * (tft.height() / 6) + 3, (tft.width() / 3) - 6,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 10 + tft.width() / 3, tft.height()/3 + tft.height()/12  + 3 * tft.height() / 6 - 11);
  tft.print("0");
  tft.fillRect((2 * tft.width() / 3) + 3, (tft.height() / 3) + 3 * (tft.height() / 6) + 3, (tft.width() / 3) - 3,(tft.height() / 6) - 3, WHITE);
  tft.setCursor(tft.width()/6 - 30 + 2 * + tft.width() / 3, tft.height()/3 + tft.height()/12  + 3 * tft.height() / 6 - 11);
  tft.print("Set");
}

void print_int_set_speed(void)
{
  tft.setTextSize(4);
  tft.setTextColor(WHITE);
  tft.fillRect(0, tft.height() / 6, tft.width(), tft.height() / 6, BLACK);
  if(interactive_set_speed > 999)
  {
    tft.setCursor(tft.width()/6 - 46 + tft.width() / 3, tft.height()/3 - 2 * tft.height()/12);
    tft.print(interactive_set_speed);
  }
  else if(interactive_set_speed > 99)
  {
    tft.setCursor(tft.width()/6 - 34 + tft.width() / 3, tft.height()/3 - 2 * tft.height()/12);
    tft.print(interactive_set_speed);
  }
  else if(interactive_set_speed > 9)
  {
    tft.setCursor(tft.width()/6 - 22 + tft.width() / 3, tft.height()/3 - 2 * tft.height()/12);
    tft.print(interactive_set_speed);
  }
  else
  {
    tft.setCursor(tft.width()/6 - 10 + tft.width() / 3, tft.height()/3 - 2 * tft.height()/12);
    tft.print(interactive_set_speed);
  }
}


