#include "SPI.h"
#include <Encoder.h>
#include <Arduino.h>
#include <iostream>
#include "Vector.h"
#include "SPI_MSTransfer_T4/SPI_MSTransfer_MASTER.h"

// SPI_MSTransfer_MASTER<&SPI, 9, 0x4567> PB2;
SPI_MSTransfer_MASTER<&SPI,9, 0x0001> PB1;
SPI_MSTransfer_MASTER<&SPI,10, 0x0002> PB2;
// class PB1:public SPI_MSTransfer_MASTER<&SPI,10, 0x0002>{};
// class PB2:public SPI_MSTransfer_MASTER<&SPI,3, 0x0003>{};
using namespace std;

int pins[3][5]={  {14,15,16,23,22},
                  {3,4,5,21,20},
                  {6,7,8,19,18}             
                
                };//dir,pwm,slp,a_vout_a,vout_b
// int b = 0;
// int* a = &b;
// *a = 1;
// spi* s;
// s->asd()
// s->a;

// func(&arr)



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
    float Angle;
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

    float get_angle(long encoder_val)//將encoder讀到的值轉成實際角度
    {
      Angle=float(encoder_val)/ pulsePerRotation * 360;
      return Angle;
    }

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

    // void run()
    // {
    //   int pwmValue = fabs(int(pwm));
    //   if (pwmValue > 255){
    //     pwmValue = 255;
    //   }
    //   board.analogWrite(pwmpin,pwmValue);
    //   board.digitalWrite(dirpin,dir);
    // }

  //   void get_SPI()
  //   {
  //     board.onTransfer(_GET_SPI);
  //   }

  //   void brake();
    
  // private:
  //   void _GET_SPI(uint16_t *buffer, uint16_t length, AsyncMST info){
  //     Serial.print(" --> PacketID: "); Serial.println(buffer[0]);
  //     if (buffer[0] == mPID){
  //       get_angle(buffer[0]);
  //     }
  //   }
};

motor motor1(0 , pins[0]);
motor motor2(1 , pins[1]);
motor motor3(2 , pins[2]);
motor motor4(0 , pins[0]);
motor motor5(1 , pins[1]);
motor motor6(2 , pins[2]);

void run_motor(auto PB,auto motor){
  // Serial.println(motor.pwm);
  PB.analogWrite(motor.pwmpin,motor.pwm);
  PB.digitalWrite(motor.dirpin,motor.dir);
}

void send_angle(auto* PB,motor * motor,float angle,int PB_ID){
  // Serial.println(motor.pwm);
  motor->dir = (angle>=0) ? LOW :HIGH ;
  uint16_t buf[3] = {motor->mPID,fabs(int(angle)),motor->dir};
  motor->target_angle=angle;
  Serial.print("true target");
  Serial.println(motor->target_angle);
  PB->transfer16(buf, sizeof(buf),PB_ID);
  PB->events();
}

int i=0;
void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {
  // i++;
  // if (i>=1000){
    // Serial.print((info.packetID));
    // Serial.print(" ");
    // Serial.print(buffer[0]);
    // Serial.print(" ");
    // Serial.print(buffer[1]);
    // Serial.print(" ");
    // Serial.println(buffer[2]);
    // Serial.println();
  //   i=0;
  // }

  if(int(info.packetID)==0){
    if(motor1.mPID == buffer[0]){
      motor1.Angle=buffer[1]*(1+(-2)*buffer[2]);
      // Serial.print("motor1: ");
      // Serial.print(buffer[1]);
      // Serial.print(" ");
      // Serial.println(buffer[0]);
    }
    if(motor2.mPID == buffer[3]){
      motor2.Angle=buffer[4]*(1+(-2)*buffer[5]);
    }
    if(motor3.mPID == buffer[6]){
      motor3.Angle=buffer[7]*(1+(-2)*buffer[8]);
    }
  }

    if(int(info.packetID)==1){
      if(motor4.mPID == buffer[0]){
        motor4.Angle=buffer[1]*(1+(-2)*buffer[2]);
      }
      if(motor5.mPID == buffer[3]){
        motor5.Angle=buffer[4]*(1+(-2)*buffer[5]);
      }
      if(motor6.mPID == buffer[6]){
        motor6.Angle=buffer[7]*(1+(-2)*buffer[8]);
      }
  }
  
}



void setup() {
  Serial.begin(9600);
  SPI.begin();
  PB1.begin();
  PB2.begin();
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  // PB1.digitalWrite

  // PB1.pinMode(0, OUTPUT);
  // PB1.pinMode(1, OUTPUT);
  // PB1.pinMode(2, OUTPUT);
  // PB1.pinMode(3, OUTPUT);
  // PB1.pinMode(4, OUTPUT);
  // PB1.pinMode(5, OUTPUT);
  // PB1.pinMode(6, OUTPUT);
  // PB1.pinMode(7, OUTPUT);
  // PB1.pinMode(8, OUTPUT);
  // PB1.pinMode(14, OUTPUT);
  // PB1.pinMode(15, OUTPUT);
  // PB1.pinMode(16, OUTPUT);
  // PB1.pinMode(18, INPUT);
  // PB1.pinMode(19, INPUT);
  // PB1.pinMode(20, INPUT);
  // PB1.pinMode(21, INPUT);
  // PB1.pinMode(22, INPUT);
  // PB1.pinMode(23, INPUT);
  
  // PB2.pinMode(0, OUTPUT);
  // PB2.pinMode(1, OUTPUT);
  // PB2.pinMode(2, OUTPUT);
  // PB2.pinMode(3, OUTPUT);
  // PB2.pinMode(4, OUTPUT);
  // PB2.pinMode(5, OUTPUT);
  // PB2.pinMode(6, OUTPUT);
  // PB2.pinMode(7, OUTPUT);
  // PB2.pinMode(8, OUTPUT);
  // PB2.pinMode(18, INPUT);
  // PB2.pinMode(19, INPUT);
  // PB2.pinMode(20, INPUT);
  // PB2.pinMode(21, INPUT);
  // PB2.pinMode(22, INPUT);
  // PB2.pinMode(23, INPUT);
  PB1.pinMode(9,OUTPUT);
  PB2.pinMode(9,OUTPUT);
  // PB1.digitalWrite(16, HIGH);
  // PB1.digitalWrite(5, HIGH);
  // PB1.digitalWrite(8, HIGH);

  PB1.onTransfer(myCB);
  PB2.onTransfer(myCB);
  
  
}
bool test=true;
bool lock=false,menu_msg=true,show_angle=false,set_angle=false,show_msg=true;
String str;
char* result;
const char * spliter=",";
float val=0;
float angleTargettmp1=0,angleTargettmp2=0,angleTargettmp3=0,angleTargettmp4=0,angleTargettmp5=0,angleTargettmp6=0;
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
      Serial.print("motor1's angle: ");
      Serial.println(motor1.Angle);
      Serial.print("motor2's angle: ");
      Serial.println(motor2.Angle);
      Serial.print("motor3's angle: ");
      Serial.println(motor3.Angle);
      Serial.print("motor4's angle: ");
      Serial.println(motor4.Angle);
      Serial.print("motor5's angle: ");
      Serial.println(motor5.Angle);
      Serial.print("motor6's angle: ");
      Serial.println(motor6.Angle);
      Serial.println();
    }
  }

  if(set_angle){
    if(show_msg){
      // Serial.println("Enter 0 to exit ");
      Serial.println("Now you can enter motor1's angle");
      show_msg=false; 
    }
  }

  while(Serial.available()>0){
    // Serial.print("Enter Value: ");
    // Serial.println(Serial.read());
    val=Serial.parseFloat();
    Serial.flush();
    Serial.println(val);

    if(!show_angle && !set_angle){
      if(int(val)==1){
        show_angle=true;
        Serial.println("show angle.....");
        lock=true;
      }
      
      if(int(val)==2){
        set_angle=true;
        Serial.println("set angle.....");
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
    
    if(set_angle){
      Serial.println(lock);
      if(lock==false){
        if (motor1.target_angle==angleTargettmp1){
          Serial.println("Now you can enter motor2's angle");
          angleTargettmp1=val;
          Serial.println(angleTargettmp1);
          Serial.flush();
          lock=true;
        }else{
          if(angleTargettmp2==motor2.target_angle && angleTargettmp1!=motor1.target_angle){
            Serial.println("Now you can enter motor3's angle");
            angleTargettmp2=val;
            Serial.flush();
            lock=true;
          }else{
            if(angleTargettmp3==motor3.target_angle && angleTargettmp2!=motor2.target_angle  && angleTargettmp1!=motor1.target_angle){
              Serial.println("Now you can enter motor4's angle");
              angleTargettmp3=val;
              Serial.flush();
              lock=true;
            }else{
              if(angleTargettmp4==motor4.target_angle && angleTargettmp3!=motor3.target_angle && angleTargettmp2!=motor2.target_angle  && \
                  angleTargettmp1!=motor1.target_angle){
                Serial.println("Now you can enter motor5's angle");
                angleTargettmp4=val;
                Serial.flush();
                lock=true;
              }else{
                if( angleTargettmp5==motor5.target_angle && angleTargettmp4!=motor4.target_angle && angleTargettmp3!=motor3.target_angle && \
                    angleTargettmp2!=motor2.target_angle  && angleTargettmp1!=motor1.target_angle){
                  Serial.println("Now you can enter motor6's angle");
                  angleTargettmp5=val;
                  Serial.flush();
                  lock=true;
                }else{
                  if(angleTargettmp6==motor6.target_angle && angleTargettmp5!=motor5.target_angle && angleTargettmp4!=motor4.target_angle && angleTargettmp3!=motor3.target_angle && \
                    angleTargettmp2!=motor2.target_angle  && angleTargettmp1!=motor1.target_angle){
                    Serial.println("ALL DONE");
                    angleTargettmp6=val;
                    Serial.flush();
                    
                    send_angle(&PB1,&motor1,angleTargettmp1,1);
                    send_angle(&PB1,&motor2,angleTargettmp2,1);
                    send_angle(&PB1,&motor3,angleTargettmp3,1);
                    send_angle(&PB2,&motor4,angleTargettmp4,2);
                    send_angle(&PB2,&motor5,angleTargettmp5,2);
                    send_angle(&PB2,&motor6,angleTargettmp6,2);
                    show_msg=true;
                    menu_msg=true;
                    set_angle=false;
                    Serial.println();
                    Serial.print("motor1 targe tangle: ");
                    Serial.println(motor1.target_angle);
                    Serial.print("motor2 targe tangle: ");
                    Serial.println(motor2.target_angle);
                    Serial.print("motor3 targe tangle: ");
                    Serial.println(motor3.target_angle);
                    Serial.print("motor4 targe tangle: ");
                    Serial.println(motor4.target_angle);
                    Serial.print("motor5 targe tangle: ");
                    Serial.println(motor5.target_angle);
                    Serial.print("motor6 targe tangle: ");
                    Serial.println(motor6.target_angle);
                    // Serial.println(angleTarget3);
                    Serial.println();
                  }                 
                }
              }
            }
          }
        }
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