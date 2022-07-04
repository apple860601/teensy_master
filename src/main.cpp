#include "SPI.h"
#include <Encoder.h>
#include <Arduino.h>
#include <iostream>
#include "Vector.h"
#include "SPI_MSTransfer_T4/SPI_MSTransfer_MASTER.h"

SPI_MSTransfer_MASTER<&SPI,4, 0x0001> PB1;
SPI_MSTransfer_MASTER<&SPI,5, 0x0002> PB2;
SPI_MSTransfer_MASTER<&SPI,6, 0x0003> PB3;

// Vector<auto> SPI_SLAVE;


int motor_num=9;
int slave_num=3;

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
    float target_angle=0;
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

  // for(int ID=0;ID<2;ID++){
    // if(int(info.packetID)==0){
  for(int i=0;i<3;i++){
    if(motor_list[3*(int(info.packetID))+i]->mPID==buffer[0]){
      motor_list[3*(int(info.packetID))+i]->Angle=buffer[1+i*3]*(1+(-2)*buffer[2+i*3]);
    }
  }
    // }
  // }
}

motor * max_motor_num[18];

int l;
void setup() {
  Serial.begin(9600);
  SPI.begin();
  PB1.begin();
  PB2.begin();
  PB3.begin();
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(4, 1);
  digitalWrite(5, 1);
  digitalWrite(6, 1);
  PB1.pinMode(9,OUTPUT);
  PB2.pinMode(9,OUTPUT);
  PB3.pinMode(9,OUTPUT);

  PB1.onTransfer(myCB);
  PB2.onTransfer(myCB);
  PB3.onTransfer(myCB);
  motor_list.setStorage(max_motor_num);

  for ( int i=0 ; i < motor_num ; i++){
    motor *motor1 = new motor(motor_sid,pins[motor_sid]);
    motor_list.push_back(motor1);
    motor_sid=(motor_sid==2)? 0:motor_sid+1;

  }
  
}


int test=0;
bool lock=false,menu_msg=true,show_angle=false,set_angle=false,show_msg=true,motor_set_sig=false;
String str,spt_str;
char* result;
const char * spliter=",";
float val=0;
float angleTargettmp1=0,angleTargettmp2=0,angleTargettmp3=0,angleTargettmp4=0,angleTargettmp5=0,angleTargettmp6=0;
int motor_select=-1,comma_pos=0;
bool relative=false;
bool array_input=false;
bool array_message=false;
Vector<float> angle_arr;
float max_angle_num[18];
bool show_target=false;



void loop() {
  PB1.events();
  PB2.events();
  PB3.events();
  angle_arr.setStorage(max_angle_num);
  static uint32_t t = millis();

  //顯示main menu
  if(menu_msg) {
    Serial.println("which service do you want to use? 1:monitor motors ; 2:set motors angle ;");
    menu_msg=false;
  }

  //顯示monitor motor資訊
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
        // Serial.println();
      }
    }
  }


  if(set_angle){

    if(show_target){
      Serial.println();
      for (int i=0;i<motor_num;i++){
        Serial.println(String("")+"Motor"+(i+1)+"'s target angle is "+motor_list[i]->target_angle);
      }
      Serial.println();
      show_target=false;
    }

    //顯示set angle menu資訊
    if(show_msg){
      Serial.println("which motor do you want to set? 1 mean motor1 \n -1:exit \n -2:switch absolute/relate angle \n -3:multimotor input \n -4:show current motor's target value");
      show_msg=false; 
    }

    //顯示array input資訊
    if (array_message){
      Serial.println("Please input array");
      Serial.println("Attention : array 's elements quantities must be same as motor quantities");
      Serial.println("eg : input 6 motor 's angle wiil be like 100,-100,150,0,12.7,-12.7 ");
      array_message=false;
    }
  }


  while(Serial.available()>0){

    //處理輸入數值
    if(array_input && set_angle){
      str=Serial.readString();
      if(str.toFloat()==-1){
        array_input=false;
        show_msg=true;
        break;
      }
      
      while (str.indexOf(",",comma_pos)!=-1)
      {
        spt_str=str.substring(comma_pos,str.indexOf(",",comma_pos));
        angle_arr.push_back(spt_str.toFloat());
        comma_pos=str.indexOf(",",comma_pos)+1;
      }
      
      angle_arr.push_back(str.substring(comma_pos,str.length()).toFloat());
      comma_pos=0;
      motor_set_sig=true;
      if (int(angle_arr.size())!=motor_num){
        Serial.println("Array's elements quantities not same as motors' quantities!!");
        motor_set_sig=false;
      }
    }else{
      val=Serial.parseFloat();
    }
    Serial.flush();

    // 處理array input
    if(set_angle  && array_input && motor_set_sig){
      // Serial.print("Array size: ");
      // Serial.println(sizeof(angle_arr));

      for (int i=0;i<int(angle_arr.size());i++){
        if(i<3){
          send_angle(&PB1,motor_list[i],angle_arr[i],1,relative);
        }else if(i<6){
          send_angle(&PB2,motor_list[i],angle_arr[i],2,relative);
        }else if(i<9){
          send_angle(&PB3,motor_list[i],angle_arr[i],3,relative);
        }
      }
      motor_select=-1;
      show_msg=true;
      motor_set_sig=false;
      array_input=false;
      break;
    }

    //處理single input
    if(set_angle && motor_set_sig && motor_select!=-1 && !array_input){
      if(motor_select<3){
        send_angle(&PB1,motor_list[motor_select],val,1,relative);
      }
      else if(motor_select<6){
        send_angle(&PB2,motor_list[motor_select],val,2,relative);
      }
      else if(motor_select<9){
        send_angle(&PB3,motor_list[motor_select],val,3,relative);
      }
      motor_select=-1;
      show_msg=true;
      motor_set_sig=false;
      break;
    }

    //處理set motor menu 選項
    if(set_angle && !motor_set_sig){
      // Serial.println(lock);
      if(val==-1){
        show_msg=true;
        menu_msg=true;
        set_angle=false;
      }else if( val>0 && val<=motor_num){
        motor_set_sig=true;
        motor_select=int(val-1);
        Serial.println(String("")+"current motor" + (motor_select+1) + "'target angle is " + motor_list[motor_select]->target_angle);
        // Serial.print(motor_select+1);
        // Serial.print("'target angle is ");
        // Serial.println(motor_list[motor_select]->target_angle);
        Serial.println("Please enter the angle");
      }else if(val==-2){
        relative=!relative;
        if(relative) Serial.println("already change to relative mode");
        else Serial.println("already change to absulte mode");
        show_msg=true;
      }else if (val==-3){
        array_input=true;
        array_message=true;
        // if(array_input) Serial.println("array input");
        // else Serial.println("single input");
        // show_msg=true;
      }else if (val==-4){
        show_target=true;
        show_msg=true;
      }else{
        Serial.println("Motor doesn't exist");
        show_msg=true;
      }
    }

    //處理main menu
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

    //處理離開monitor angle部分
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
    
    // Serial.println(test);
    PB1.digitalWrite(9,test==0);
    PB2.digitalWrite(9,test==1);
    PB3.digitalWrite(9,test==2);
    test++;
    if(test==3) test=0;
    t = millis();
  }
  delay(1);
}