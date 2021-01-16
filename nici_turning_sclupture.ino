#include "kal/kal.h"
//#include<Wire.h>

#define DEBUG 1

//wave
kal::wave sin_wave(0.0,PI/2,0.5,SIN);

//robot
#define MOTOR_NUM 1
kal::nxtmotor motor[MOTOR_NUM];

//differentiator
kal::Diff<double> dtheta_st[MOTOR_NUM];
kal::Diff<double> dtheta_ref[MOTOR_NUM];

//udp通信
kal::udp_for_esp32<kal::q_data> udp0(ISOLATED_NETWORK);

//時間管理//@todo: freeRTOSの導入検討
double t = 0.0;//time
bool timer_flag = 0;

//timer関連
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {  /* this function must be placed in IRAM */
  portENTER_CRITICAL_ISR(&timerMux);
  //control---------------------------------------------------------------------------------------------------------------------------/
  t += Ts;
  timer_flag = 1; 
  //-----------------------------------------------------------------------------------------------------------------------------------/
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  Serial.println("started");
  
  //motor1の設定
  motor[0].GPIO_setup(GPIO_NUM_25,GPIO_NUM_26);//方向制御ピン設定
  motor[0].PWM_setup(GPIO_NUM_12,0);//PWMピン設定
  motor[0].encoder_setup(PCNT_UNIT_0,GPIO_NUM_36,GPIO_NUM_39);//エンコーダカウンタ設定
  motor[0].set_fb_v_param(10.0,1.0,0.0);
  motor[0].set_fb_param(30,0.5,5.0);
  motor[0].set_fb_cc_param(50.0,0.0);

//  //motor2
//  motor[1].GPIO_setup(GPIO_NUM_16,GPIO_NUM_17);//方向制御ピン設定
//  motor[1].PWM_setup(GPIO_NUM_15,0);//PWMピン設定
//  motor[1].encoder_setup(PCNT_UNIT_1,GPIO_NUM_34,GPIO_NUM_35);//エンコーダカウンタ設定
//  motor[1].set_fb_v_param(10.0,1.0,0.0);
//  motor[1].set_fb_param(30,0.0,5.0);
//  motor[1].set_fb_cc_param(50.0,0.0);
//  //motor3
//  motor[2].GPIO_setup(GPIO_NUM_5,GPIO_NUM_21);//方向制御ピン設定
//  motor[2].PWM_setup(GPIO_NUM_13,0);//PWMピン設定
//  motor[2].encoder_setup(PCNT_UNIT_2,GPIO_NUM_32,GPIO_NUM_33);//エンコーダカウンタ設定
//  //motor4
//  motor[3].GPIO_setup(GPIO_NUM_22,GPIO_NUM_23);//方向制御ピン設定
//  motor[3].PWM_setup(GPIO_NUM_12,0);//PWMピン設定
//  motor[3].encoder_setup(PCNT_UNIT_3,GPIO_NUM_25,GPIO_NUM_26);//エンコーダカウンタ設定

//UDP通信設定
  udp0.set_udp(esp_ssid,esp_pass);

  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts*1000000), true);//Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化
}
void loop() {

  //udp受信
  char c = udp0.receive_char();
  if(timer_flag){//制御周期
    timer_flag = 0;

    //状態取得---------------------------------------------------------------------------//
  
    for(int i=0;i<MOTOR_NUM;i++){
      motor[i].get_angle(motor[i].state.q);  
    }
    for(int i=0;i<MOTOR_NUM;i++){
      dtheta_st[i].update(motor[i].state.q,motor[i].state.dq);  
    }
    //------------------------------------------------------------------------------------//
    
    //目標値計算
    sin_wave.update();
    motor[0].ref.q = sin_wave.output;
    dtheta_ref[0].update(motor[0].ref.q,motor[0].ref.dq);
  
    //出力計算
    double u = motor[0].position_control();
    motor[0].drive(u);
//    motor[0].drive(sin_wave.output);

//  udp0.send_char(',');
//  delay(100);
      
#if DEBUG
    for(int i=0;i<MOTOR_NUM;i++){
      Serial.print(motor[i].ref.q * RAD2DEG);
      Serial.print(",");
      Serial.print(motor[i].state.q * RAD2DEG);     
      Serial.print(",");      
    }
    Serial.println();
#endif

  }//制御周期
  else{//その他の処理
    
  }
}
