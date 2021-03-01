#include "kal/kal.h"
#define DEBUG 1

//waveの形定義
kal::wave wave0(0.0,PI/2,1.0/360.0,TRIANGLE);
//kal::wave wave0(0.0,PI/6,1.0/60.0,TRIANGLE);
kal::wave wave_pwm(0.0,3.0,0.1,SIN);

//motorの定義
#define MOTOR_NUM 1
kal::nxtmotor motor[MOTOR_NUM];

//differentiator微分器の定義
kal::Diff<double> dtheta_st[MOTOR_NUM];
kal::Diff<double> dtheta_ref[MOTOR_NUM];

//touch switchタッチスイッチの定義
bool touch_switch = 0;
#define TOUCH_SWITCH_PIN GPIO_NUM_32

//udp通信の定義
kal::udp_for_esp32<kal::q_data> udp0(ISOLATED_NETWORK);

//時間管理
double t = 0.0;//time
bool timer_flag = 0;

//timerの設定
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {//時間計測
  portENTER_CRITICAL_ISR(&timerMux);
  //control----------------------------------------------------------------------------//
  t += Ts;
  timer_flag = 1; 
  //-----------------------------------------------------------------------------------//
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  Serial.println("started");
  
  //motor1の設定
  motor[0].GPIO_setup(GPIO_NUM_25,GPIO_NUM_26);//方向制御ピン設定
  motor[0].PWM_setup(GPIO_NUM_12,0,50000,10);//PWMピン設定
  motor[0].encoder_setup(PCNT_UNIT_0,GPIO_NUM_39,GPIO_NUM_36);//エンコーダカウンタ設定
  motor[0].set_fb_param(40,0.0,5.0);

//  //motor2 2個目のモータを使う場合
//  motor[1].GPIO_setup(GPIO_NUM_16,GPIO_NUM_17);//方向制御ピン設定
//  motor[1].PWM_setup(GPIO_NUM_15,0);//PWMピン設定
//  motor[1].encoder_setup(PCNT_UNIT_1,GPIO_NUM_34,GPIO_NUM_35);//エンコーダカウンタ設定
//  motor[1].set_fb_param(30,0.0,5.0);

  //タッチスイッチの設定
  pinMode(TOUCH_SWITCH_PIN,INPUT);

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

    //センサの値取得---------------------------------------------------------------------------//
    //角度センサ
    for(int i=0;i<MOTOR_NUM;i++){
      motor[i].get_angle(motor[i].state.q);  
    }
    //角速度計算
    for(int i=0;i<MOTOR_NUM;i++){
      dtheta_st[i].update(motor[i].state.q,motor[i].state.dq);  
    }
    //タッチスイッチの値取得 1:pushed, 0:not pushed
    touch_switch = digitalRead(TOUCH_SWITCH_PIN);  
    //------------------------------------------------------------------------------------//
    
    //目標値計算----------------------------------------------------------------------------//
    wave0.update();
    wave_pwm.update();
    motor[0].ref.q = wave0.output;
    dtheta_ref[0].update(motor[0].ref.q,motor[0].ref.dq);
    //-----------------------------------------------------------------------------------//
    
    //出力計算----------------------------------------------------------------------------//
    double u = motor[0].position_control();//pid位置制御
//    double u = wave_pwm.output;

    if( c=='o' ){
      u = 2.0;
    }
    else if( c=='c'){
      u = -2.0;
    }
    else if( c=='e'){
      u = 0.0;
    }
    motor[0].drive(u);
    
//  udp0.send_char(',');
//  delay(100);
//-----------------------------------------------------------------------------------//
      
#if DEBUG//グラフで確認用
    for(int i=0;i<MOTOR_NUM;i++){
      Serial.print(motor[i].ref.q * RAD2DEG);
      Serial.print(",");
      Serial.print(motor[i].state.q * RAD2DEG);     
      Serial.print(",");
      Serial.print(u);
      Serial.print(",");  
//      Serial.print(touch_switch);      
    }
    Serial.println();
#endif

  }//制御周期
  else{//その他の処理
    
  }
}
