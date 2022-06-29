#include "SPI.h"
#include <Encoder.h>
#include <Arduino.h>
#include <iostream>
#include "Vector.h"
#include "SPI_MSTransfer_T4/SPI_MSTransfer_MASTER.h"

SPI_MSTransfer_MASTER<&SPI,9, 0x0001> PB1;
SPI_MSTransfer_MASTER<&SPI,10, 0x0002> PB2;

int motor_num=6;
int slave_num=2;

using namespace std;

int pins[3][5]={  {14,15,16,23,22},
                  {3,4,5,21,20},
                  {6,7,8,19,18}             
                
                };//dir,pwm,slp,vout_a,vout_b


class motor // 單一馬達的類別
{

  public:
    int mPID;
    bool dir;
    int dirpin;
    int pwmpin;
    int slppin;
    int encoder_a_pin;
    int encoder_b_pin;
    float pwm;
    float Angle=0;
    float errorAngle;
    float integralErrorAngle;
    float pulsePerRotation=480;
    float Ki=0.15,Kp=0.9,Kd=0.05;
    float deltaTime = 0.010; 
    float target_angle;
    motor a();
    motor b();
    motor c();
    motor() {};

    ~motor() {};

    motor(motor a,motor b,motor c){a=a;b=b;c=c;};

    motor(int pid,int *pins)//建立motor類別;所需參數:pid->馬達在該版的編號、pins->馬達在該版需要的腳位
    {
      mPID=pid;
      dirpin=*pins;
      pwmpin=*pins+1;
      slppin=*pins+2;
      encoder_a_pin=*pins+3;
      encoder_b_pin=*pins+4;
    };

    motor(int pid,int *pins,float ki,float kp,float kd)
    {
      mPID=pid;
      dirpin=*pins;
      pwmpin=*pins+1;
      slppin=*pins+2;
      encoder_a_pin=*pins+3;
      encoder_b_pin=*pins+4;
      Ki=ki;
      Kp=kp;
      Kd=kd;
    };

    void generate_control_signal(float target,bool relative)//生成pwm訊號與dir訊號
    { 
      target_angle=target;
      float new_errAngle = target_angle - Angle;
      float errorAngleDot = (new_errAngle - errorAngle) / deltaTime;
      float new_integralErrorAngle = integralErrorAngle + new_errAngle * deltaTime;
      float controlSignal = Kp * new_errAngle + Kd * errorAngleDot + Ki*integralErrorAngle;

      if (controlSignal>0) dir=LOW;
      else dir=HIGH;

      errorAngle=new_errAngle;
      pwm=controlSignal;
      integralErrorAngle=new_integralErrorAngle;
    }
};

Vector<motor*> motor_list;
int motor_sid=0;

void run_motor(auto PB,auto motor){

  PB.analogWrite(motor.pwmpin,motor.pwm);
  PB.digitalWrite(motor.dirpin,motor.dir);

}

void send_angle(auto* PB,motor * motor,float angle,int PB_ID,bool relative){

  if (relative) motor->target_angle=motor->target_angle+angle;
  else motor->target_angle=angle;
  motor->dir = (motor->target_angle>0) ? LOW :HIGH ;
  uint16_t buf[3] = {motor->mPID,fabs(int(motor->target_angle)),motor->dir};
  PB->transfer16(buf, sizeof(buf),PB_ID);
  PB->events();
}

int i=0;
void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {

  for(int ID=0;ID<2;ID++){
    if(int(info.packetID)==0){
      for(int i=0;i<3;i++){
        if(motor_list[3*ID+i]->mPID==buffer[0]){
          motor_list[3*ID+i]->Angle=buffer[1+i*3]*(1+(-2)*buffer[2+i*3]);
        }
      }
    }
  }
}

motor * max_motor_num[18];

int l;
void setup() {
  Serial.begin(9600);
  SPI.begin();
  PB1.begin();
  PB2.begin();
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  PB1.pinMode(9,OUTPUT);
  PB2.pinMode(9,OUTPUT);

  PB1.onTransfer(myCB);
  PB2.onTransfer(myCB);
  motor_list.setStorage(max_motor_num);

  for ( int i=0 ; i < motor_num ; i++){
    motor *motor1 = new motor(motor_sid,pins[motor_sid]);
    motor_list.push_back(motor1);
    motor_sid=(motor_sid==2)? 0:motor_sid+1;
    // Serial.println(motor_list[i]->mPID);

  }
  
}


bool test=true;
bool lock=false,menu_msg=true,show_angle=false,set_angle=false,show_msg=true,motor_set_sig=false;
String str;
char* result;
const char * spliter=",";
float val=0;
float angleTargettmp1=0,angleTargettmp2=0,angleTargettmp3=0,angleTargettmp4=0,angleTargettmp5=0,angleTargettmp6=0;
int motor_select=-1;
bool relative=false;

void loop() {
  PB1.events();
  PB2.events();
  static uint32_t t = millis();
  if(menu_msg) {
    Serial.println("which service do you want to use? 1:monitor motors ; 2:set motors angle ;");
    menu_msg=false;
  }
  if(show_angle){
    if(show_msg){
      Serial.println("Enter -1 to exit ");
      Serial.println("monitoring motors..... ");
      Serial.println();
      show_msg=false; 
    }
    if ( millis() - t > 1000 ) {
      for (int i=0 ; i<motor_num ; i++){
        Serial.print("motor");
        Serial.print(i+1);
        Serial.print("'s angle: ");
        Serial.println((motor_list[i])->Angle);
        // Serial.println(0.0001);
        Serial.println();
      }
    }
  }

  if(set_angle){
    if(show_msg){
      Serial.println("which motor do you want to set? 1 mean motor1 ; -1:exit ; -2:switch absolute/relate angle");
      show_msg=false; 
    }
  }

  while(Serial.available()>0){
    val=Serial.parseFloat();
    Serial.flush();
    // Serial.println(val);

    if(set_angle && motor_set_sig && motor_select!=-1){
      motor_set_sig=false;
      if(motor_select<3){
        send_angle(&PB1,motor_list[motor_select],val,1,relative);
      }
      else if(motor_select<6){
        send_angle(&PB2,motor_list[motor_select],val,2,relative);
      }
      motor_select=-1;
      show_msg=true;
      break;
    }

    if(set_angle && !motor_set_sig){
      // Serial.println(lock);
      if(val==-1){
        show_msg=true;
        menu_msg=true;
        set_angle=false;
      }else if( val>0 && val<motor_num){
        motor_set_sig=true;
        motor_select=int(val-1);
        Serial.print("current motor");
        Serial.print(motor_select+1);
        Serial.print("'target angle is ");
        Serial.println(motor_list[motor_select]->target_angle);
        Serial.println("Please enter the angle");
      }else if(val==-2){
        relative=!relative;
        if(relative) Serial.println("already change to relative mode");
        else Serial.println("already change to absulte mode");
        show_msg=true;
      }else{
        Serial.println("Motor doesn't exist");
        show_msg=true;
      }
    }

    if(!show_angle && !set_angle){
      if(int(val)==1){
        show_angle=true;
        Serial.println("showing angle.....");
        lock=true;
      }
      
      if(int(val)==2){
        set_angle=true;
        Serial.println("setting angle.....");
        lock=true;
      }
    }

    if(show_angle){
      switch (int(val))
      {
      case -1:
        show_angle=false;
        menu_msg=true;
        show_msg=true;
        break;
      }
    }

    Serial.read();
  }
  if (!Serial.available()){
    lock=false;
  }

  if ( millis() - t > 1000 ) {
    test=!test;
    PB1.digitalWrite(9,test);
    PB2.digitalWrite(9,!test);
    t = millis();
  }
  delay(1);
}