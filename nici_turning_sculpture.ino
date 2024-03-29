#include "kal/kal.h"
#define DEBUG 1

//ROLL, PITCH, YAWの装置選択,選択したtype以外をコメントアウト
//#define ROLL
//#define PITCH
#define YAW

//一定時間リセットの設定
#define RESET_TIME 3600.0
//緊急停止の設定
#define STOP_TIME 3.0
#define EMERGENCY_COUNTER (STOP_TIME/Ts) 
int stop_cnt = 0;

//mode設定
#define PID_MODE//精密制御モード
kal::LPF<double> lpf(0.0,0.5);//目標値にLPFかけて滑らかにする
double ref_lpf = 0.0;

//nici 設定パラメータ
//ROLL type------------------------------------------------------------------------------//
#ifdef ROLL//ROLL のパラメータ
//基本的にこの2つだけで調整できる
#define MAX_ANGLE 120.0//最大角度(ライトの振幅)[度]
#define TIME 240.0//1往復にかかる時間[秒]
//さらに細かい調整
#define V_NORMAL 2.0//motorにかける電圧[V](motorの回転速度)
#define LIMIT (DEG2RAD*30)//ライトの振れ幅の最低点とスイッチの距離kirikomitani限界90[s]
kal::wave wave0(0.0,MAX_ANGLE*DEG2RAD,1.0/TIME,-PI/2.0,TRIANGLE);
//char ssid[] = "ROLL_wifi"; // SSID
//PITCH type------------------------------------------------------------------------------//
#elif defined PITCH//PITCH のパラメータ
//基本的にこの2つだけで調整できる
#define MAX_ANGLE 20.0//最大角度(ライトの振幅)[度]
#define TIME 70.0//1往復にかかる時間[秒]kirikomitani限界90[s]
//さらに細かい調整
#define V_NORMAL 2.0//motorにかける電圧[V](motorの回転速度)
#define LIMIT (DEG2RAD*5)//ライトの振れ幅の最低点とスイッチの距離
kal::wave wave0(0.0,MAX_ANGLE*DEG2RAD,1.0/TIME,-PI/2.0,TRIANGLE);
//char ssid[] = "PITCH_wifi"; // SSID
//YAW type------------------------------------------------------------------------------//
#elif defined YAW//YAW のパラメータ
//基本的にこの2つだけで調整できる
#define MAX_ANGLE 80.0//最大角度(ライトの振幅)[度]
#define TIME 80.0//1往復にかかる時間[秒]kirikomitani限界90[s]
//さらに細かい調整
#define V_NORMAL -4.2//motorにかける電圧[V](motorの回転速度)
#define LIMIT (DEG2RAD*30)//ライトの振れ幅の最低点とスイッチの距離
kal::wave wave0(0.0,MAX_ANGLE*DEG2RAD,1.0/TIME,PI/2.0,TRIANGLE);
//char ssid[] = "YAW_wifi"; // SSID
#endif

//kal zone --------------------------------------------------------------------------------//
//状態管理
#define INITIALIZE_STATE 0
#define DRIVING_STATE 1
#define EMERGENCY_STATE 2
int state = INITIALIZE_STATE;
double offset_angle = 0.0;
double angle = 0.0;
#define AMP (MAX_ANGLE*DEG2RAD)//回転の振幅[rad]

//回転方向のstate
#define CW 0//順回転
#define CCW 1//逆回転
int turn_direction_state = CW;

//motorの定義
#define MOTOR_NUM 1
kal::nxtmotor motor[MOTOR_NUM];

//differentiator微分器の定義
kal::Diff<double> dtheta_st[MOTOR_NUM];
kal::Diff<double> dtheta_ref[MOTOR_NUM];

//touch switchタッチスイッチの定義
#define TOUCH_SWITCH_PIN GPIO_NUM_32
bool touch_switch = 0;
double initialized_time;

//udp通信の定義
//kal::udp_for_esp32<kal::q_data> udp0(ISOLATED_NETWORK);

//時間管理
double t = 0.0;//time
bool timer_flag = 0;

//timerの設定
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {//時間計測
  portENTER_CRITICAL_ISR(&timerMux);
  //control----------------------------------------//
  t += Ts;
  timer_flag = 1; 
  //-----------------------------------------------//
  portEXIT_CRITICAL_ISR(&timerMux);
}
//kal zone end ------------------------------------------------------------------------//

//初期設定
void setup() {
  Serial.begin(115200);
  Serial.println("start");
  
  //motor1の設定
  motor[0].GPIO_setup(GPIO_NUM_25,GPIO_NUM_26);//方向制御ピン設定
  motor[0].PWM_setup(GPIO_NUM_12,0,50000,10);//PWMピン設定
  motor[0].encoder_setup(PCNT_UNIT_0,GPIO_NUM_39,GPIO_NUM_36);//エンコーダカウンタ設定
#ifdef ROLL
  motor[0].set_fb_param(40.0,0.0,2.0);//PID制御のパラメータ設定
  motor[0].set_ff_param(0.0,0.0,0.0,0.78);//FF制御のパラメータ設定
  Serial.println("ROLL");
#elif defined PITCH
  motor[0].set_fb_param(50.0,0.0,3.5);//PID制御のパラメータ設定
  motor[0].set_ff_param(0.0,0.0,0.0,0.78);//FF制御のパラメータ設定
  Serial.println("PITCH");
#elif defined YAW
  motor[0].set_fb_param(45.0,0.0,3.5);//PID制御のパラメータ設定
  motor[0].set_ff_param(0.0,0.0,0.0,3.0);//FF制御のパラメータ設定
  Serial.println("YAW");
#endif

  //タッチスイッチの設定
  pinMode(TOUCH_SWITCH_PIN,INPUT);

  //UDP通信設定
//  udp0.set_udp(esp_ssid,esp_pass);

  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts*1000000), true);//Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化

  delay(1000);
}

//制御
void loop() {

  //udp受信
//  char c = udp0.receive_char();
  if(timer_flag){//制御周期
    timer_flag = 0;
    if(t>=RESET_TIME){
      t = 0.0;
//      state=INITIALIZE_STATE;
      if(state!=EMERGENCY_STATE){
        ESP.restart(); 
      }
    }

    //センサの値取得------------------------------------------------------------------------//
    //角度センサ
    for(int i=0;i<MOTOR_NUM;i++){
      motor[i].get_angle(motor[i].state.q);  
    }
    angle = motor[0].state.q + offset_angle;
    //角速度計算
    for(int i=0;i<MOTOR_NUM;i++){
      dtheta_st[i].update(motor[i].state.q,motor[i].state.dq);  
    }
    //タッチスイッチの値取得 1:pushed, 0:not pushed
    touch_switch = digitalRead(TOUCH_SWITCH_PIN);  
    //-----------------------------------------------------------------------------------//
    
    //kiriko mitani zone ----------------------------------------------------------------//
    double u = 0.0;//u:motor power[v]

    //初期位置設定state
    if(state==INITIALIZE_STATE){
      Serial.println("Initializing");
      u = -V_NORMAL;//タッチスイッチの位置まで回転
      initialized_time = t;
      if(touch_switch==1){//タッチスイッチが押されたら
        u = 0.0;
        state = DRIVING_STATE;//運転stateに移行
        turn_direction_state=CW;//逆回転モード設定
        offset_angle = -motor[0].state.q-LIMIT-AMP;//motorの角度設定を変更
        lpf.y = motor[0].state.q;
        lpf.x = motor[0].state.q;
        initialized_time = t;
#ifdef YAW
        wave0.ave = motor[0].state.q - wave0.amp - LIMIT;
#else
        wave0.ave = motor[0].state.q + wave0.amp + LIMIT;
#endif
        Serial.println("Start Driving");
      }
    }
    
    //運転中state
    else if(state==DRIVING_STATE){
#ifdef PID_MODE
      //目標値計算---------------------------------------------------------------------------//
      wave0.update();
      lpf.update(wave0.output,ref_lpf);
      motor[0].ref.q = ref_lpf;//wave0.output;
      dtheta_ref[0].update(motor[0].ref.q,motor[0].ref.dq);
      //-----------------------------------------------------------------------------------//
      //出力計算----------------------------------------------------------------------------//
      u = motor[0].two_dof_control();
      //-----------------------------------------------------------------------------------//
#else
      if(turn_direction_state==CW){//順方向回転モード
        Serial.println("CW");
        u = V_NORMAL;//順方向回転
        if(angle>AMP){//角度が振幅より大きくなったら逆回転モード
          turn_direction_state=CCW;
        }
      }
      else if(turn_direction_state==CCW){//逆方向回転モード
        Serial.println("CCW");
        u = -V_NORMAL;//逆回転
        if(angle<-AMP){//角度が振幅より大きくなったら逆回転モード
          turn_direction_state=CW;
        }
      }
#endif
    }
    //koriko mitani zone end -------------------------------------------------------------//
    
//    //wifi制御
//    if( c=='o' ){
//      u = 2.0;
//    }
//    else if( c=='c'){
//      u = -2.0;
//    }
//    else if( c=='e'){
//      u = 0.0;    }

    //緊急停止モード
    if(state==EMERGENCY_STATE){
      u = 0.0;
      Serial.println("EMERGENCY_STATE");
    }
    
    motor[0].drive(u);//motorへ決定した電圧を出力
    
//  udp0.send_char(',');
//  delay(100);
//-----------------------------------------------------------------------------------//

    //スイッチに当たった場合
    if(touch_switch==1 && t>initialized_time+10.0){//タッチスイッチが押されたら
        u = 0.0;
        state = DRIVING_STATE;//運転stateに移行
        turn_direction_state=CW;//逆回転モード設定
        offset_angle = -motor[0].state.q-LIMIT-AMP;//motorの角度設定を変更
        lpf.y = motor[0].state.q;
        lpf.x = motor[0].state.q;
        initialized_time = t;
#ifdef YAW
        wave0.ave = motor[0].state.q - wave0.amp - LIMIT;
#else
        wave0.ave = motor[0].state.q + wave0.amp + LIMIT;
#endif
      }

    //モーターの速度監視
    if(abs(motor[0].state.dq)<0.01){
      stop_cnt++;
    }
    else{
      stop_cnt = 0;
    }
    if(stop_cnt>=EMERGENCY_COUNTER){
      state = EMERGENCY_STATE;
      stop_cnt = EMERGENCY_COUNTER;
    }
      
#if DEBUG//グラフで確認用
    for(int i=0;i<MOTOR_NUM;i++){
//      Serial.print(t);
//      Serial.print(",");
#ifdef PID_MODE
      Serial.print((motor[i].ref.q-wave0.ave) * RAD2DEG);
      Serial.print(",");
      Serial.print((motor[i].state.q-wave0.ave) * RAD2DEG); 
#else 
      Serial.print(angle * RAD2DEG);     
#endif
      Serial.print(",");
      Serial.print(motor[i].state.dq);
      Serial.print(",");
      Serial.print(stop_cnt);
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
