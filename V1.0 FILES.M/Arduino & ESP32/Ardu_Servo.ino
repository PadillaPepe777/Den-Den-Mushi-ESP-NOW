//Ardu-Servos for Den Den Mushi
#include <Servo.h>

#define L_EyeServo 3
#define R_EyeServo 5
#define TalkServo 6

#define Act_Eye 7
#define TalkIN 8

Servo L_Eye;
Servo R_Eye;
Servo Talk;

const int Sleep=42;
const int Wake=3;
int CurrentEye=0;
int CurrentEyeR=0;
const int Silence=50;
const int Talking=0;
int CurrentTalk=0;


const int BlinkTime=1800;
const int TalkTime=1200;

unsigned long BlinkTimer;
unsigned long TalkTimer;



void setup() {
  Serial.begin(9600);

  pinMode(Act_Eye, INPUT_PULLUP);
  pinMode(TalkIN, INPUT_PULLUP);

  L_Eye.attach(L_EyeServo);
  R_Eye.attach(R_EyeServo);
  Talk.attach(TalkServo);

  BlinkTimer=millis();
  TalkTimer=millis();
}

void loop() {

  if(!digitalRead(TalkIN))
  {
    CurrentTalk=Talking;
    //Talkingloop();
  }
  else
    CurrentTalk=Silence;
  
  if(!digitalRead(Act_Eye))
    Blinking();
  else
  {
    CurrentEye=Sleep;
    CurrentEyeR=Wake;
  }

  Talk.write(CurrentTalk);
  L_Eye.write(CurrentEye);
  R_Eye.write(CurrentEyeR);
  delay(20);

}

void Talkingloop (){

if(millis()-TalkTimer>=TalkTime)
{
  if(CurrentTalk==Silence)
  CurrentTalk=Talking;
  else
  CurrentTalk=Silence;

  TalkTimer=millis();
}

}

void Blinking (){

if((millis()-BlinkTimer>=BlinkTime) && CurrentEye==Wake)
{
  CurrentEye=Sleep;
  CurrentEyeR=Wake;
  BlinkTimer=millis();
}
else if((millis()-BlinkTimer>=150) && CurrentEye==Sleep)
{
  CurrentEye=Wake;
  CurrentEyeR=Sleep;
  BlinkTimer=millis();
}

}
