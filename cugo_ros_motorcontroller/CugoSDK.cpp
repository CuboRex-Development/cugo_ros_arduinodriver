#include "CugoSDK.h"
#include "Arduino.h"
#include <math.h>

RPI_PICO_Timer ITimer0(0);

// グローバル変数宣言
int cugo_old_runmode = CUGO_RC_MODE;
int cugo_runmode = CUGO_RC_MODE;
bool cugo_switching_reset = true;
long int cugo_count_prev_L = 0;
long int cugo_count_prev_R = 0;

//カウント関連
long int cugo_target_count_L = 0;
long int cugo_target_count_R = 0;
long int cugo_start_count_L = 0;
long int cugo_start_count_R = 0;
long int cugo_current_count_L = 0;
long int cugo_current_count_R = 0;
volatile long cugo_current_encoder_R = 0;
volatile long cugo_current_encoder_L = 0;
long int cugo_prev_encoder_L = 0;
long int cugo_prev_encoder_R = 0;

unsigned long long int cugo_calc_odometer_time = 0;
float cugo_odometer_theta = 0;
float cugo_odometer_x = 0;
float cugo_odometer_y = 0;
float cugo_odometer_degree = 0;
long int cugo_odometer_count_theta = 0;
bool cugo_direction_L = true;
bool cugo_direction_R = true;



//ld2関連
volatile long cugo_ld2_id = 0;
volatile long cugo_ld2_feedback_hz = 0;
volatile long cugo_ld2_feedback_dutation = 0;
const long int cugo_ld2_index_tofreq[3] = { 10, 50, 100 };

//ボタン関連
//int cugo_button_count = 0;
//bool cugo_button_check = true;
//volatile unsigned long long cugo_button_start_time = 0;


//各種関数
//初期化関数
void cugo_init() {

  Serial.begin(115200, SERIAL_8N1);   //PCとの通信
  Serial1.begin(115200, SERIAL_8N1);  //BLDCとの通信

  delay(1000);  //LD-2起動待機
  return_all_data();

  //ボタン関連
  //pinMode(CUGO_CMD_BUTTON_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(CUGO_CMD_BUTTON_PIN),cugo_button_interrupt, CHANGE);

  //ld2_set_feedback(2, 0b10000001);  //freq{0:10[hz] 1:50[hz] 2:100[hz]} kindof_data{0b1:Mode 0b10:CMD_RPM 0b100:CurrentRPM 0b1000:AveCurrentRPM 0b10000000:EncorderData}
  delay(1000);
  if (!(ITimer0.attachInterruptInterval(cugo_ld2_feedback_dutation * 1000, cugo_timer_handler0))) {
    Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }
  delay(1000);
  ld2_set_control_mode(CUGO_CMD_MODE);
  cugo_reset_odometer();
  delay(1000);
  cugo_init_display();

  //ボタン関連
  //cugo_reset_button_times();
}

bool cugo_timer_handler0(struct repeating_timer* t) {
  ld2_get_cmd();
  return true;
}

//ボタン関連
/*
void cugo_button_interrupt(){
  if(digitalRead(CUGO_CMD_BUTTON_PIN) == HIGH){
  cugo_button_count++;
  cugo_button_check = true;
  cugo_button_start_time = micros();
  }
  if(digitalRead(CUGO_CMD_BUTTON_PIN) == LOW){
  cugo_button_start_time = 0;
  cugo_button_check = false;
  }

}
*/
void cugo_wait(unsigned long long int wait_ms) {
  //約70分まで計測可能
  //例１時間待機したい場合：cugo_wait(120UL*60UL*1000UL);
  unsigned long long int cugo_target_wait_time = wait_ms * 1000ull;
  unsigned long long int MAX_MICROS = 4294967295ull;  // micros() 関数で計測する最大値
  unsigned long long int startMicros = 0;             // 計測開始時刻
  unsigned long long int elapsedMicros = 0;           // 経過時間（マイクロ秒単位）
  unsigned long long int currentMicros = micros();

  if (cugo_target_wait_time < MAX_MICROS) {
    while (elapsedMicros < cugo_target_wait_time) {

      if (startMicros == 0) {
        startMicros = micros();  // 計測開始時刻が初期化されていない場合、初期化する
      }
      currentMicros = micros();  //時刻を取得

      // 経過時間を計算する
      if (currentMicros >= startMicros) {
        elapsedMicros = currentMicros - startMicros;
      } else {
        // オーバーフローが発生した場合
        elapsedMicros = (MAX_MICROS - startMicros) + currentMicros + 1;
      }
    }
  } else {
    Serial.println(F("##WARNING::cugo_waitの計測可能時間を超えています。##"));
  }
}

void cugo_long_wait(unsigned long long int wait_seconds) {
  //例 24時間計測したい場合：cugo_wait(24UL*60UL*60UL);

  unsigned long long int cugo_target_wait_time = wait_seconds * 1000ull;
  unsigned long long int MAX_MILLIS = 4294967295ull;  // millis() 関数で計測する最大値
  unsigned long long int startMillis = 0;             // 計測開始時刻
  unsigned long long int elapsedMillis = 0;           // 経過時間（マイクロ秒単位）
  unsigned long long int currentMillis = millis();

  if (cugo_target_wait_time < MAX_MILLIS) {

    while (elapsedMillis < cugo_target_wait_time) {
      if (startMillis == 0) {
        startMillis = millis();  // 計測開始時刻が初期化されていない場合、初期化する
      }
      currentMillis = millis();  //現在時刻を取得

      // 経過時間を計算する
      if (currentMillis >= startMillis) {
        elapsedMillis = currentMillis - startMillis;
      } else {
        // オーバーフローが発生した場合
        elapsedMillis = (MAX_MILLIS - startMillis) + currentMillis + 1;
      }
    }
  } else {
    Serial.println(F("##WARNING::cugo_long_waitの計測可能時間を超えています。##"));
  }
}

void cugo_move_pid(float target_rpm, bool use_pid) {
  ld2_encoder_reset();
  cugo_start_odometer();

  //PID制御値
  float l_count_p = 0;
  float l_count_i = 0;
  float l_count_d = 0;
  float r_count_p = 0;
  float r_count_i = 0;
  float r_count_d = 0;
  // PID位置制御のデータ格納
  float l_count_prev_i_ = 0;
  float l_count_prev_p_ = 0;
  //float l_count_prev_p_ = (cugo_target_count_L -      /*/*.getCount()*/*/)/10000.0;
  float r_count_prev_i_ = 0;
  float r_count_prev_p_ = 0;
  //float r_count_prev_p_ = (cugo_target_count_R -      /*/*.getCount()*/*/)/10000.0 ;
  float l_count_gain = 0;
  float r_count_gain = 0;

  if (target_rpm <= 0) {
    Serial.println(F("##WARNING::目標速度が0以下のため進みません##"));
  } else if (cugo_target_count_L == 0 && cugo_target_count_R == 0) {
    Serial.println(F("##WARNING::目標距離が左右ともに0のため進みません##"));
  } else {

    if (cugo_target_count_L >= 0) {
      cugo_direction_L = true;
    } else {
      cugo_direction_L = false;
    }
    if (cugo_target_count_R >= 0) {
      cugo_direction_R = true;
    } else {
      cugo_direction_R = false;
    }

    if (!use_pid) {
      if (cugo_direction_L) {
        l_count_gain = target_rpm;
      } else {
        l_count_gain = -target_rpm;
      }
      if (cugo_direction_R) {
        r_count_gain = target_rpm;
      } else {
        r_count_gain = -target_rpm;
      }
    }
    if (abs(target_rpm) > CUGO_MAX_MOTOR_RPM) {
      Serial.println(F("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##"));
    }

    //cugo_test時確認用
    //Serial.print("target_count L/R:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));


    while (!cugo_check_count_achievement(CUGO_MOTOR_LEFT) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT)) {
      if (cugo_target_count_L == 0 && cugo_target_count_R == 0) {
        cugo_rpm_direct_instructions(0, 0);
      } else {
        if (use_pid) {
          // 各制御値の計算
          //★
          l_count_p = cugo_target_count_L - cugo_current_count_L;
          l_count_i = l_count_prev_i_ + l_count_p;
          l_count_d = l_count_p - l_count_prev_p_;
          //★
          r_count_p = cugo_target_count_R - cugo_current_count_R;
          r_count_i = r_count_prev_i_ + r_count_p;
          r_count_d = r_count_p - r_count_prev_p_;

          l_count_i = min(max(l_count_i, -CUGO_L_MAX_COUNT_I), CUGO_L_MAX_COUNT_I);
          r_count_i = min(max(r_count_i, -CUGO_R_MAX_COUNT_I), CUGO_R_MAX_COUNT_I);
          // PID制御
          l_count_gain = (l_count_p * CUGO_L_COUNT_KP + l_count_i * CUGO_L_COUNT_KI + l_count_d * CUGO_L_COUNT_KD);
          r_count_gain = (r_count_p * CUGO_R_COUNT_KP + r_count_i * CUGO_R_COUNT_KI + r_count_d * CUGO_R_COUNT_KD);
          // prev_ 更新
          l_count_prev_p_ = l_count_p;
          l_count_prev_i_ = l_count_i;
          r_count_prev_p_ = r_count_p;
          r_count_prev_i_ = r_count_i;
          l_count_gain = min(max(l_count_gain, -CUGO_MAX_MOTOR_RPM), CUGO_MAX_MOTOR_RPM);  //モーターの速度上限
          r_count_gain = min(max(r_count_gain, -CUGO_MAX_MOTOR_RPM), CUGO_MAX_MOTOR_RPM);  //モーターの速度上限
          l_count_gain = min(max(l_count_gain, -fabsf(target_rpm)), fabsf(target_rpm));    //ユーザ設定の速度上限
          r_count_gain = min(max(r_count_gain, -fabsf(target_rpm)), fabsf(target_rpm));    //ユーザ設定の速度上限

          //位置制御
          if (cugo_check_count_achievement(CUGO_MOTOR_LEFT)) {
            l_count_gain = 0;
          }
          if (cugo_check_count_achievement(CUGO_MOTOR_RIGHT)) {
            r_count_gain = 0;
          }
        }
      }
      //Serial.println("gain l:r:" + String(l_count_gain)+" ,"+ String(l_count_gain));
      cugo_rpm_direct_instructions(l_count_gain, r_count_gain);
      cugo_wait(10);
      cugo_calc_odometer();
    }

    //cugo_test時確認用
    //Serial.println("result_count L/R:" + String(cugo_current_count_L)+" ,"+ String(cugo_current_count_R));

    //Serial.println(F("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));
    //Serial.println(F("result_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));
    //Serial.println(F("==========="));
  }
  cugo_stop();
  ld2_encoder_reset();
}

//前進制御＆回転制御
void cugo_move_forward(float target_distance) {
  cugo_calc_necessary_count(target_distance);
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM, true);
}
void cugo_move_forward(float target_distance, float target_rpm) {
  cugo_calc_necessary_count(target_distance);
  cugo_move_pid(target_rpm, true);
}
void cugo_move_forward_raw(float target_distance, float target_rpm) {
  cugo_calc_necessary_count(target_distance);
  cugo_move_pid(target_rpm, false);
}
void cugo_turn_clockwise(float target_degree) {
  cugo_calc_necessary_rotate(target_degree);
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM, true);
}
void cugo_turn_clockwise(float target_degree, float target_rpm) {
  cugo_calc_necessary_rotate(target_degree);
  cugo_move_pid(target_rpm, true);
}
void cugo_turn_clockwise_raw(float target_degree, float target_rpm) {
  cugo_calc_necessary_rotate(target_degree);
  cugo_move_pid(target_rpm, false);
}
void cugo_turn_counterclockwise(float target_degree) {
  cugo_calc_necessary_rotate(-target_degree);
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM, true);
}
void cugo_turn_counterclockwise(float target_degree, float target_rpm) {
  cugo_calc_necessary_rotate(-target_degree);
  cugo_move_pid(target_rpm, true);
}
void cugo_turn_counterclockwise_raw(float target_degree, float target_rpm) {
  cugo_calc_necessary_rotate(-target_degree);
  cugo_move_pid(target_rpm, false);
}
//円軌道での移動命令
void cugo_curve_theta_raw(float target_radius, float target_degree, float target_rpm) {
  ld2_encoder_reset();
  cugo_start_odometer();

  float target_rpm_L = 0;
  float target_rpm_R = 0;

  cugo_target_count_L = (target_radius - CUGO_TREAD / 2) * (target_degree * PI / 180) * (CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI));
  cugo_target_count_R = (target_radius + CUGO_TREAD / 2) * (target_degree * PI / 180) * (CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI));

  if (target_rpm <= 0) {
    Serial.println(F("##WARNING::目標速度が0以下のため進みません##"));
  } else if (cugo_target_count_L == 0 && cugo_target_count_R == 0) {
    Serial.println(F("##WARNING::目標距離が左右ともに0のため進みません##"));
  } else {
    if (target_radius < 0) {
      cugo_target_count_L = -cugo_target_count_L;
      cugo_target_count_R = -cugo_target_count_R;
    }
    if (target_degree > 0) {
      if (target_radius != 0) {
        target_rpm_L = target_rpm * ((target_radius - CUGO_TREAD / 2) / target_radius);
        target_rpm_R = target_rpm * ((target_radius + CUGO_TREAD / 2) / target_radius);
      } else {
        target_rpm_L = -target_rpm;
        target_rpm_R = target_rpm;
      }
    } else if (target_degree < 0) {
      if (target_radius != 0) {
        target_rpm_L = -target_rpm * ((target_radius - CUGO_TREAD / 2) / target_radius);
        target_rpm_R = -target_rpm * ((target_radius + CUGO_TREAD / 2) / target_radius);
      } else {
        target_rpm_L = target_rpm;
        target_rpm_R = -target_rpm;
      }
    } else {  //target_degree=0
      target_rpm_L = 0;
      target_rpm_R = 0;
    }

    if (abs(target_rpm_L) > CUGO_MAX_MOTOR_RPM || abs(target_rpm_R) > CUGO_MAX_MOTOR_RPM) {
      Serial.println(F("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##"));
    }

    if (abs(target_rpm_L) < 20 || abs(target_rpm_R) < 20) {
      Serial.println(F("##WARNING::目標速度が十分な速度ではないため正確な軌道を進まない可能性があります。##"));
    }

    if (cugo_target_count_L >= 0) {
      cugo_direction_L = true;
    } else {
      cugo_direction_L = false;
    }

    if (cugo_target_count_R >= 0) {
      cugo_direction_R = true;
    } else {
      cugo_direction_R = false;
    }

    target_rpm_L = min(max(target_rpm_L, -CUGO_MAX_MOTOR_RPM), CUGO_MAX_MOTOR_RPM);  //モーターの速度上限
    target_rpm_R = min(max(target_rpm_R, -CUGO_MAX_MOTOR_RPM), CUGO_MAX_MOTOR_RPM);  //モーターの速度上限


    //cugo_test時確認用
    //cugo_test時確認用
    //Serial.print("target_count L/R:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));

    while (!cugo_check_count_achievement(CUGO_MOTOR_LEFT) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT)) {
      if (cugo_target_count_L == 0 && cugo_target_count_R == 0) {
        target_rpm_L = 0;
        target_rpm_R = 0;
      }
      if (cugo_check_count_achievement(CUGO_MOTOR_LEFT)) {
        target_rpm_L = 0;
      }
      if (cugo_check_count_achievement(CUGO_MOTOR_RIGHT)) {
        target_rpm_R = 0;
      }
      cugo_rpm_direct_instructions(target_rpm_L, target_rpm_R);
      cugo_wait(10);
      cugo_calc_odometer();
    }

    //cugo_test時確認用
    //Serial.println("result_count L/R:" + String(cugo_current_count_L)+" ,"+ String(cugo_current_count_R));
  }
  cugo_stop();
  ld2_encoder_reset();
}
void cugo_curve_distance_raw(float target_radius, float target_distance, float target_rpm) {
  ld2_encoder_reset();
  cugo_start_odometer();

  float target_rpm_L = 0;
  float target_rpm_R = 0;

  if (target_radius == 0) {
    Serial.println(F("##WARNING::軌道半径が0のため進みません##"));
  } else {
    cugo_target_count_L = target_distance * ((target_radius - CUGO_TREAD / 2) / target_radius) * ((CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)));
    cugo_target_count_R = target_distance * ((target_radius + CUGO_TREAD / 2) / target_radius) * ((CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)));

    if (target_rpm <= 0) {
      Serial.println(F("##WARNING::目標速度が0以下のため進みません##"));
    } else if (cugo_target_count_L == 0 && cugo_target_count_R == 0) {
      Serial.println(F("##WARNING::目標距離が左右ともに0のため進みません##"));
    } else {

      if (target_distance > 0) {
        if (target_radius != 0) {
          target_rpm_L = target_rpm * ((target_radius - CUGO_TREAD / 2) / target_radius);
          target_rpm_R = target_rpm * ((target_radius + CUGO_TREAD / 2) / target_radius);
        } else {
          target_rpm_L = -target_rpm;
          target_rpm_R = target_rpm;
        }
      } else if (target_distance < 0) {
        if (target_radius != 0) {
          target_rpm_L = -target_rpm * ((target_radius - CUGO_TREAD / 2) / target_radius);
          target_rpm_R = -target_rpm * ((target_radius + CUGO_TREAD / 2) / target_radius);
        } else {
          target_rpm_L = target_rpm;
          target_rpm_R = -target_rpm;
        }
      } else {  //target_distance = 0
        target_rpm_L = 0;
        target_rpm_R = 0;
      }

      if (abs(target_rpm_L) > CUGO_MAX_MOTOR_RPM || abs(target_rpm_R) > CUGO_MAX_MOTOR_RPM) {
        Serial.println(F("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##"));
      }

      if (abs(target_rpm_L) < 20 || abs(target_rpm_R) < 20) {
        Serial.println(F("##WARNING::目標速度が十分な速度ではないため正確な軌道を進まない可能性があります。##"));
      }

      if (cugo_target_count_L >= 0) {
        cugo_direction_L = true;
      } else {
        cugo_direction_L = false;
      }

      if (cugo_target_count_R >= 0) {
        cugo_direction_R = true;
      } else {
        cugo_direction_R = false;
      }

      target_rpm_L = min(max(target_rpm_L, -CUGO_MAX_MOTOR_RPM), CUGO_MAX_MOTOR_RPM);  //モーターの速度上限
      target_rpm_R = min(max(target_rpm_R, -CUGO_MAX_MOTOR_RPM), CUGO_MAX_MOTOR_RPM);  //モーターの速度上限


      //cugo_test時確認用
      //Serial.print("target_count L/R:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));

      //
      while (!cugo_check_count_achievement(CUGO_MOTOR_LEFT) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT)) {
        if (cugo_target_count_L == 0 && cugo_target_count_R == 0) {
          target_rpm_L = 0;
          target_rpm_R = 0;
        }
        if (cugo_check_count_achievement(CUGO_MOTOR_LEFT)) {
          target_rpm_L = 0;
        }
        if (cugo_check_count_achievement(CUGO_MOTOR_RIGHT)) {
          target_rpm_R = 0;
        }
        cugo_rpm_direct_instructions(target_rpm_L, target_rpm_R);
        cugo_wait(10);
        cugo_calc_odometer();
      }

      //cugo_test時確認用
      //Serial.println("result_count L/R:" + String(cugo_current_count_L)+" ,"+ String(cugo_current_count_R));
      /*  
      //Serial.println(F("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
      Serial.println(F("result_count l:r:" + String(     .getCount())+" ,"+ String(     .getCount()));
      //Serial.println(F("==========="));
      */
    }
  }
  cugo_stop();
  ld2_encoder_reset();
}
//チェック関連
bool cugo_check_count_achievement(int motor_num_) {
  //
  long int target_count_ = 0;
  long int current_count_ = 0;
  bool cugo_direction_;  //前進後進方向の変数変更
  if (motor_num_ == CUGO_MOTOR_LEFT) {
    target_count_ = cugo_target_count_L;
    cugo_direction_ = cugo_direction_L;
    current_count_ = cugo_current_count_L;
  } else if (motor_num_ == CUGO_MOTOR_RIGHT) {
    target_count_ = cugo_target_count_R;
    cugo_direction_ = cugo_direction_R;
    current_count_ = cugo_current_count_R;

  } else {
    return false;
  }
  //Serial.println(String(current_count_));


  // 目標達成チェック
  if (cugo_direction_) {
    if (target_count_ <= current_count_) {
      //    [motor_num_]//.setTargetRpm(0);
      //Serial.println("motor"+ String(motor_num_)+"done::" + String(target_count_) +","+String(current_count_));
      return true;
    }
  } else {
    if (target_count_ >= current_count_) {
      //Serial.println("motor"+ String(motor_num_)+"done::" + String(target_count_) +","+String(current_count_));
      return true;
    }
  }

  return false;
}
//ボタン関連
/*
bool cugo_check_button(){
  return cugo_button_check;  
  }
*/
//ボタン関連
/*
int cugo_check_button_times(){
  
  return cugo_button_count;  
  }
*/
//ボタン関連
/*
void cugo_reset_button_times(){
  cugo_button_count = 0;
  }
*/
//ボタン関連
/*
long int cugo_button_press_time(){
  volatile unsigned long long cugoButtonTime;
  if(cugo_button_start_time != 0){
  cugoButtonTime = micros();
  return (cugoButtonTime-cugoButtonTime)/1000;  
  }else{
  return 0;  
  }
  return 0;  
  }
*/
float cugo_check_odometer(int check_number) {
  //odometer_number 0:x,1:y,2:theta
  if (check_number == CUGO_ODO_X) {
    return cugo_odometer_x;
  } else if (check_number == CUGO_ODO_Y) {
    return cugo_odometer_y;
  } else if (check_number == CUGO_ODO_THETA) {
    return cugo_odometer_theta;
  } else if (check_number == CUGO_ODO_DEGREE) {
    return cugo_odometer_degree;
  } else {
    return 0;
  }
  return 0;
}
void cugo_calc_odometer() {
  if (micros() - cugo_calc_odometer_time > 10000) {
    long int cugo_dif_count_theta_ = (cugo_current_count_R - cugo_count_prev_R) - (cugo_current_count_L - cugo_count_prev_L);
    long int cugo_dif_count_v_ = ((cugo_current_count_R - cugo_count_prev_R) + (cugo_current_count_L - cugo_count_prev_L)) / 2;
    cugo_odometer_theta += cugo_dif_count_theta_ * ((2 * CUGO_WHEEL_RADIUS_L * PI / CUGO_ENCODER_RESOLUTION) / CUGO_TREAD);

    cugo_odometer_x += (cugo_dif_count_v_ * cos(cugo_odometer_theta)) * (2 * CUGO_WHEEL_RADIUS_L * PI / CUGO_ENCODER_RESOLUTION);
    cugo_odometer_y += (cugo_dif_count_v_ * sin(cugo_odometer_theta)) * (2 * CUGO_WHEEL_RADIUS_L * PI / CUGO_ENCODER_RESOLUTION);
    cugo_odometer_degree = cugo_odometer_theta * 180 / PI;
    cugo_count_prev_L = cugo_current_count_L;
    cugo_count_prev_R = cugo_current_count_R;
    cugo_calc_odometer_time = micros();
  }
}
void cugo_reset_odometer() {
  cugo_count_prev_L = 0;
  cugo_count_prev_R = 0;
  cugo_odometer_count_theta = 0;
  cugo_odometer_x = 0;
  cugo_odometer_y = 0;
  cugo_odometer_theta = 0;
  cugo_odometer_degree = 0;
  cugo_calc_odometer_time = micros();
}
void cugo_start_odometer() {
  cugo_count_prev_L = 0;
  cugo_count_prev_R = 0;
  cugo_calc_odometer_time = micros();
}

void cugo_calc_necessary_rotate(float degree) {
  //Serial.println(F("#   cugo_calc_necessary_rotate"));//確認用
  cugo_target_count_L = ((degree / 360) * CUGO_TREAD * PI) * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI);
  cugo_target_count_R = -((degree / 360) * CUGO_TREAD * PI) * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //Serial.println(F("degree: " + String(degree));
  //Serial.println("### cugo_target_count_L/R: " + String(cugo_target_count_L) + " / " + String(cugo_target_count_R) + "###");
  //Serial.println(F("kakudo: " + String((degree / 360) * CUGO_TREAD * PI));
  //Serial.println(F("PI: " + String(PI));
  //Serial.println(F("issyuu: " + String(2 * CUGO_WHEEL_RADIUS_R * PI));
}
void cugo_calc_necessary_count(float distance) {
  //Serial.println(F("#   cugo_calc_necessary_count"));//確認用

  //cugo_target_count_L = distance / (2 * CUGO_WHEEL_RADIUS_L * PI);
  //cugo_target_count_R = distance / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //cugo_target_count_L = cugo_target_count_L * CUGO_ENCODER_RESOLUTION;
  //cugo_target_count_R = cugo_target_count_R * CUGO_ENCODER_RESOLUTION;
  cugo_target_count_L = distance * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI);
  cugo_target_count_R = distance * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //Serial.print(F("distance: "));
  //Serial.println(String(cugo_target_count_L));



  //Serial.println(F("distance: " + String(distance));
  //Serial.println(F("distance: " + String(CUGO_ENCODER_RESOLUTION));
  //Serial.println(F("2 * CUGO_WHEEL_RADIUS_L * PI: " + String(2 * CUGO_WHEEL_RADIUS_L * PI));
  //Serial.println(F("calc: " + String(distance * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)));
  //Serial.println(F("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###"));
  //Serial.println(F("distance: " + String(distance));
  //Serial.println(F("CUGO_WHEEL_RADIUS_L: " + String(CUGO_WHEEL_RADIUS_L));
  //Serial.println(F("PI: " + String(PI));
  //Serial.println(F("issyuu: " + String(2 * CUGO_WHEEL_RADIUS_R * PI));
}
void cugo_rpm_direct_instructions(float left, float right) {
  //Serial.println(F("#   cugo_rpm_direct_instructions"));//確認用
  //Serial.println("rpm  l:r:" + String(left)+" ,"+ String(right));
  unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
  ld2_float_to_frame(left, 2, frame);
  ld2_float_to_frame(right, 6, frame);
  ld2_write_cmd(frame);
  //cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

void cugo_stop() {
  //Serial.println(F("#   cugo_stop"));//確認用
  unsigned char frame[10] = { 0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0 };
  ld2_float_to_frame(0, 2, frame);
  ld2_float_to_frame(0, 6, frame);
  ld2_write_cmd(frame);
  cugo_wait(100);  // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

void cugo_init_display() {
  //delay(30);
  cugo_wait(30);

  Serial.println(F("###############################"));
  Serial.println(F("###   CugoAruduinoKitStart  ###"));
  Serial.println(F("###############################"));
}
void cugo_test(int test_number) {
  unsigned long int cugo_test_start = micros();
  if (test_number == 0) {  //試験プログラムパターン⓪：サンプルプログラムテスト
    Serial.println(F("自動走行モード開始"));
    Serial.println(F("1.0mの正方形移動の実施"));

    cugo_move_forward(1.0);
    cugo_wait(1000);
    cugo_turn_clockwise(90, 90);
    cugo_wait(1000);
    cugo_move_forward(1.0);
    cugo_wait(1000);
    cugo_turn_clockwise(90, 90);
    cugo_wait(1000);
    cugo_move_forward(1.0);
    cugo_wait(1000);
    cugo_turn_clockwise(90, 90);
    cugo_wait(1000);
    cugo_move_forward(1.0);
    cugo_wait(1000);
    cugo_turn_clockwise(90, 90);
    cugo_wait(1000);

    Serial.println(F("自動走行モード終了"));
    cugo_runmode = CUGO_RC_MODE;
  }
  if (test_number == 1) {  //試験プログラムパターン①：走行関連テスト
    Serial.println(F("自動走行モード開始"));
    Serial.println(F("試験項目（単体 ）"));
    /*
      Serial.println(F(""));

        cugo_wait(100);
        Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
        cugo_reset_odometer();
    */
    /*
    cugo_move_forward
    cugo_move_forward_raw
    */

    Serial.println(F("cugo_move_forward(1.0)"));
    cugo_move_forward(1.0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward(1.0,60)"));
    cugo_move_forward(1.0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(1.0,60)"));
    cugo_move_forward_raw(1.0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_move_forward(-1.0)"));
    cugo_move_forward(-1.0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward(-1.0,60)"));
    cugo_move_forward(-1.0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(-1.0,60)"));
    cugo_move_forward_raw(-1.0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_move_forward(0)"));
    cugo_move_forward(0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward(0,60)"));
    cugo_move_forward(0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(0,60)"));
    cugo_move_forward_raw(0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_move_forward(10)"));
    cugo_move_forward(10);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward(10,60)"));
    cugo_move_forward(10, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(10,60)"));
    cugo_move_forward_raw(10, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_move_forward(1.0,40)"));
    cugo_move_forward(1.0, 40);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(1.0,40)"));
    cugo_move_forward_raw(1.0, 40);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_move_forward(1.0,-40)"));
    cugo_move_forward(1.0, -40);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(1.0,-40)"));
    cugo_move_forward_raw(1.0, -40);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_move_forward(1.0,0)"));
    cugo_move_forward(1.0, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(1.0,0)"));
    cugo_move_forward_raw(1.0, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    /*
    cugo_turn_clockwise
    cugo_turn_clockwise_raw
    */

    Serial.println(F("cugo_turn_clockwise(90)"));
    cugo_turn_clockwise(90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise(90,60)"));
    cugo_turn_clockwise(90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(90,60)"));
    cugo_turn_clockwise_raw(90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(-90)"));
    cugo_turn_clockwise(-90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise(-90,60)"));
    cugo_turn_clockwise(-90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(-90,60)"));
    cugo_turn_clockwise_raw(-90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(0)"));
    cugo_turn_clockwise(0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise(0,60)"));
    cugo_turn_clockwise(0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(0,60)"));
    cugo_turn_clockwise_raw(0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(540)"));
    cugo_turn_clockwise(540);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise(540,60)"));
    cugo_turn_clockwise(540, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(540,60)"));
    cugo_turn_clockwise_raw(540, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(90,20)"));
    cugo_turn_clockwise(90, 20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(90,20)"));
    cugo_turn_clockwise_raw(90, 20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(90,-20)"));
    cugo_turn_clockwise(90, -20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(90,-20)"));
    cugo_turn_clockwise_raw(90, -20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(90,0)"));
    cugo_turn_clockwise(90, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(90,0)"));
    cugo_turn_clockwise_raw(90, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    /*
    cugo_turn_counterclockwise
    cugo_turn_counterclockwise_raw
    */

    Serial.println(F("cugo_turn_counterclockwise(90)"));
    cugo_turn_counterclockwise(90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(90,60)"));
    cugo_turn_counterclockwise(90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(90,60)"));
    cugo_turn_counterclockwise_raw(90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_counterclockwise(-90)"));
    cugo_turn_counterclockwise(-90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(-90,60)"));
    cugo_turn_counterclockwise(-90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(-90,60)"));
    cugo_turn_counterclockwise_raw(-90, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_counterclockwise(0)"));
    cugo_turn_counterclockwise(0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(0,60)"));
    cugo_turn_counterclockwise(0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(0,60)"));
    cugo_turn_counterclockwise_raw(0, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(540)"));
    cugo_turn_counterclockwise(540);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(540,60)"));
    cugo_turn_counterclockwise(540, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(540,60)"));
    cugo_turn_counterclockwise_raw(540, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(90,20)"));
    cugo_turn_counterclockwise(90, 20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(90,20)"));
    cugo_turn_counterclockwise_raw(90, 20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(90,-20)"));
    cugo_turn_counterclockwise(90, -20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(90,-20)"));
    cugo_turn_counterclockwise_raw(90, -20);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(90,0)"));
    cugo_turn_counterclockwise(90, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise_raw(90,0)"));
    cugo_turn_counterclockwise_raw(90, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    /*
    cugo_curve_theta_raw
    */

    Serial.println(F("cugo_curve_theta_raw(0.3,90,90)"));
    cugo_curve_theta_raw(0.3, 90, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0.3,-90,90)"));
    cugo_curve_theta_raw(0.3, -90, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0.3,0,90)"));
    cugo_curve_theta_raw(0.3, 0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0.3,540,90)"));
    cugo_curve_theta_raw(0.3, 540, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(-0.3,90,90)"));
    cugo_curve_theta_raw(-0.3, 90, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(-0.3,-90,90)"));
    cugo_curve_theta_raw(-0.3, -90, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(-0.3,0,90)"));
    cugo_curve_theta_raw(-0.3, 0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(-0.3,540,90)"));
    cugo_curve_theta_raw(-0.3, 540, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0,90,90)"));
    cugo_curve_theta_raw(0, 90, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0,-90,90)"));
    cugo_curve_theta_raw(0, -90, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0,0,90)"));
    cugo_curve_theta_raw(0, 0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0,540,90)"));
    cugo_curve_theta_raw(0, 540, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(1.0,30,-90)"));
    cugo_curve_theta_raw(1.0, 30, -90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(1.0,30,0)"));
    cugo_curve_theta_raw(1.0, 30, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    /*
    cugo_curve_distance_raw
    */

    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,90)"));
    cugo_curve_distance_raw(1.0, 1.0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(1.0,-1.0,90)"));
    cugo_curve_distance_raw(1.0, -1.0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(1.0,0,90)"));
    cugo_curve_distance_raw(1.0, 0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(1.0,18,90)"));
    cugo_curve_distance_raw(1.0, 18, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(-1.0,1,90)"));
    cugo_curve_distance_raw(-1.0, 1, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(-1.0,-1,90)"));
    cugo_curve_distance_raw(-1.0, -1, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(-1.0,0,90)"));
    cugo_curve_distance_raw(-1.0, 0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(-1.0,18,90)"));
    cugo_curve_distance_raw(-1.0, 18, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(0,1,90)"));
    cugo_curve_distance_raw(0, 1, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(0,-1,90)"));
    cugo_curve_distance_raw(0, -1, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(0,0,90)"));
    cugo_curve_distance_raw(0, 0, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(0,18,90)"));
    cugo_curve_distance_raw(0, 18, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(1,1,-90)"));
    cugo_curve_distance_raw(1, 1, -90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_distance_raw(1,1,0)"));
    cugo_curve_distance_raw(1, 1, 0);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("自動走行モード終了"));
  }
  if (test_number == 2) {  //試験プログラムパターン②：プロポ入力、ボタン関連テスト
    Serial.println(F("自動走行モード開始"));
    Serial.println(F("試験項目（耐久）"));
    //試験用関数記載
    Serial.println(F("cugo_move_forward(100,120)"));
    cugo_move_forward(100, 120);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(100,180)"));
    cugo_move_forward(100, 180);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward(-100,120)"));
    cugo_move_forward(-100, 120);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_move_forward_raw(-100,180)"));
    cugo_move_forward(-100, 180);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_clockwise(5400,60)"));
    cugo_turn_clockwise(5400, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_clockwise_raw(5400,180)"));
    cugo_turn_clockwise_raw(5400, 180);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_turn_counterclockwise_raw(5400,60)"));
    cugo_turn_counterclockwise_raw(5400, 60);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_turn_counterclockwise(5400,180)"));
    cugo_turn_counterclockwise(5400, 180);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_curve_theta_raw(50,180,90)"));
    cugo_curve_theta_raw(50, 180, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();
    Serial.println(F("cugo_curve_theta_raw(0,1800,90)"));
    cugo_curve_theta_raw(0, 1800, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    Serial.println(F("cugo_curve_distance_raw(50,100,90)"));
    cugo_curve_distance_raw(50, 100, 90);
    cugo_wait(100);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_reset_odometer();

    cugo_wait(100);
    Serial.println(F("自動走行モード終了"));
  }
  if (test_number == 3) {  //試験プログラムパターン③：半径1.0mのS字移動
    Serial.println(F("自動走行モード開始"));
    Serial.println(F("半径1.0mのS字移動"));
    //試験用関数記載
    cugo_curve_theta_raw(1.0, 180, 90);
    cugo_wait(1000);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    cugo_curve_theta_raw(-1.0, 180, 90);
    cugo_wait(1000);
    Serial.println("X : " + String(cugo_check_odometer(CUGO_ODO_X)) + ", Y : " + String(cugo_check_odometer(CUGO_ODO_Y)) + ", DEGREE: " + String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("自動走行モード終了"));
  }
  if (test_number == 4) {  //試験プログラムパターン④ランダム試験
    Serial.println(F("自動走行モード開始"));
    Serial.println(F("試験項目（ランダム）"));

    //試験用関数記載

    /*
    Serial.println(F("cugo_move_forward(0.5)"));
      cugo_move_forward(0.5);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_move_forward(-0.5)"));
      cugo_move_forward(-0.5);      
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_move_forward(0)"));
      cugo_move_forward(0); 
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));    
    Serial.println(F("cugo_move_forward_raw(0.5,70)"));
      cugo_move_forward_raw(0.5,70);      
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_move_forward_raw(-0.5,40)"));
      cugo_move_forward_raw(-0.5,40);      
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise(90)"));
      cugo_turn_clockwise(90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise(-90)"));
      cugo_turn_clockwise(-90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise(0)"));
      cugo_turn_clockwise(0); 
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise(90,90)"));
      cugo_turn_clockwise(90,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise(90,0)"));
      cugo_turn_clockwise(90,0);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise(90,-90)"));
      cugo_turn_clockwise(90,-90); 
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise_raw(60,80)"));
      cugo_turn_clockwise_raw(60,80);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_clockwise_raw(-60,40)"));
      cugo_turn_clockwise_raw(-60,40);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise(90)"));
      cugo_turn_counterclockwise(90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise(-90)"));
      cugo_turn_counterclockwise(-90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise(0)"));
      cugo_turn_counterclockwise(0);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise(90,90)"));
      cugo_turn_counterclockwise(90,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise(90,0)"));
      cugo_turn_counterclockwise(90,0);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise(90,-90)"));
      cugo_turn_counterclockwise(90,-90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise_raw(60,80)"));
      cugo_turn_counterclockwise_raw(60,80);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    Serial.println(F("cugo_turn_counterclockwise_raw(-60,40)"));
      cugo_turn_counterclockwise_raw(-60,40);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(1.0,90,90)"));
      cugo_curve_theta_raw(1.0,90,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(-1.0,90,90)"));
      cugo_curve_theta_raw(-1.0,90,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(0,90,90)"));
      cugo_curve_theta_raw(0,90,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(0.5,540,90)"));
      cugo_curve_theta_raw(0.5,540,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(1.0,-90,90)"));
      cugo_curve_theta_raw(1.0,-90,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(1.0,0,90)"));
      cugo_curve_theta_raw(1.0,0,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(1.0,90,180)"));
      cugo_curve_theta_raw(1.0,90,180);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(1.0,90,-90)"));
      cugo_curve_theta_raw(1.0,90,-90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_theta_raw(1.0,90,0)"));
      cugo_curve_theta_raw(1.0,90,0);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,90)"));
      cugo_curve_distance_raw(1.0,1.0,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(-1.0,1.0,90)"));
      cugo_curve_distance_raw(-1.0,1.0,90);    
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(0,1.0,90)"));
      cugo_curve_distance_raw(0,1.0,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(0.5,12.0,90)"));
      cugo_curve_distance_raw(0.5,12.0,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(1.0,-1.0,90)"));
      cugo_curve_distance_raw(1.0,-1.0,90);    
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(1.0,0,90)"));
      cugo_curve_distance_raw(1.0,0,90);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,180)"));
      cugo_curve_distance_raw(1.0,1.0,180);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,-90)"));
      cugo_curve_distance_raw(1.0,1.0,-90);    
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));

    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,0)"));
      cugo_curve_distance_raw(1.0,1.0,0);
      cugo_wait(100);
      Serial.println("X : "+ String(cugo_check_odometer(CUGO_ODO_X)) +", Y : "+String(cugo_check_odometer(CUGO_ODO_Y))+", DEGREE: "+String(cugo_check_odometer(CUGO_ODO_DEGREE)));
    */

    /*
    //試験用関数記載

    target_time_test = 60*60*1000;
    Serial.println(F("test_time"+String(target_time_test));
    cugo_long_wait(target_time_test);
    Serial.println(String(target_time_test));
    Serial.println(F("done!"));
    cugo_turn_clockwise(90,-20);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_clockwise(90,0);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_counterclockwise(90,-20);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_counterclockwise(90,-20);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_theta_raw(1.0,30,-90);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 

    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_theta_raw(1.0,30,0);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println(F("自動走行モード終了")); 
    Serial.println(F("自動走行モード開始"));  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_distance_raw(0,18,90);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
        Serial.println(F("start"));  
        cugo_long_wait(7200);
        Serial.println(F("done"));  

    */
    Serial.println(F("自動走行モード終了"));
  }
  Serial.println("処理時間(micros)::" + String(micros() - cugo_test_start));
  ld2_set_control_mode(CUGO_RC_MODE);
}


//------------------------------------便利関数
void ld2_float_to_frame(float data, long int start, unsigned char* index) {  //配列indexの4番目からfloat dataを書き込む場合-> FloatToInt(data, 4, index);
  memcpy(&index[start], &data, 4);
}
void ld2_frame_to_float(unsigned char* index, long int start, float* data) {  //配列indexの3番目からfloat dataに書き込む場合-> ld2_frame_to_float(index, 3, data);
  memcpy(data, &index[start], 4);
}
void ld2_frame_to_short(unsigned char* index, long int start, short* data) {  //配列indexの3番目からuint16_t dataに書き込む場合-> ld2_frame_to_float(index, 3, data);
  memcpy(data, &index[start], 2);
}

//------------------------------------通信関係
void ld2_write_cmd(unsigned char cmd[10]) {  //引数はidとチェックサム以外の配列
  long int i;
  unsigned char checksum = cugo_ld2_id;
  for (i = 0; i < 10; i++) {
    Serial1.write(cmd[i]);
    //Serial.print(cmd[i],HEX);
    //Serial.print(",");
    checksum += (unsigned char)(cmd[i]);
  }
  Serial1.write(cugo_ld2_id);
  //Serial.print(id,HEX);
  //Serial.print(",");
  Serial1.write(checksum);
  //Serial.println(checksum,HEX);
  cugo_ld2_id++;
  if (cugo_ld2_id > 0xFF) cugo_ld2_id = 0;
}

void ld2_get_cmd() {  //引数はidとチェックサム以外の配列
  unsigned char frame[12];
  while (Serial1.available() >= 12) {
    while (Serial1.read() != 0xFF) {
    }
    frame[0] = 0xFF;
    //Serial.print(String(frame[0]));
    //Serial.print(",");
    for (long int i = 1; i < 12; i++) {
      frame[i] = Serial1.read();
      //  Serial.print(String(frame[i]));
      //  Serial.print(",");
    }
    //  Serial.println("");
    /*
    if(frame[1] == 0x8E){  //5.5.6 Encoder Feedback
      ld2_set_encorder(frame);

    }*/

    if (frame[1] == 0x80) {  //5.5.1 Control Mode Feedback
      if (frame[2] == 0x00) {
        if (cugo_switching_reset) {
          if (cugo_old_runmode == CUGO_CMD_MODE) {
            //Serial.println(F("###   RESETTING........     ###"));
            cugo_reset();
          } else if (cugo_old_runmode == CUGO_RC_MODE) {
            cugo_runmode = CUGO_RC_MODE;
          } else {
          }
        } else {
          if (cugo_old_runmode == CUGO_CMD_MODE) {
            cugo_runmode = CUGO_RC_MODE;
            cugo_old_runmode = CUGO_RC_MODE;
            Serial.println(F("###   MODE:CUGO_RC_MODE     ###"));

          } else if (cugo_old_runmode == CUGO_RC_MODE) {
            cugo_runmode = CUGO_RC_MODE;
          } else {
          }
        }
      } else if (frame[2] == 0x01) {
        if (cugo_old_runmode == CUGO_RC_MODE) {
          cugo_runmode = CUGO_CMD_MODE;
          cugo_old_runmode = CUGO_CMD_MODE;
          Serial.println(F("###   MODE:CUGO_CMD_MODE    ###"));
        } else if (cugo_old_runmode == CUGO_CMD_MODE) {
          cugo_runmode = CUGO_CMD_MODE;
        } else {
        }
      }
    } else if (frame[1] == 0x82) {  //5.5.2 CMD RPM Feedback

    } else if (frame[1] == 0x84) {  //5.5.3 Current RPM Feedback
      //ld2_frame_to_float(frame,2,&rpm_current_L);
      //ld2_frame_to_float(frame,4,&rpm_current_R);

    } else if (frame[1] == 0x86) {  //5.5.4 Average RPM Feedback

    } else if (frame[1] == 0x8D) {  //5.5.5 SBUS Signal Feedback

    } else if (frame[1] == 0x8E) {  //5.5.6 Encoder Feedback
      ld2_set_encorder(frame);

    } else if (frame[1] == 0x8F) {  //Data Feedback Config

    } else {
    }
  }
}

void ld2_set_encorder(unsigned char frame[12]) {
  short encoderR = 0, encoderL = 0;
  ld2_frame_to_short(frame, 2, &encoderL);
  ld2_frame_to_short(frame, 4, &encoderR);

  int diff_L = encoderL - cugo_prev_encoder_L;
  int diff_R = encoderR - cugo_prev_encoder_R;
  // オーバーフロー処理
  if (diff_L < -CUGO_LD2_COUNT_MAX / 2) {
    diff_L += CUGO_LD2_COUNT_MAX;
  } else if (diff_L > CUGO_LD2_COUNT_MAX / 2) {
    diff_L -= CUGO_LD2_COUNT_MAX;
  }

  if (diff_R < -CUGO_LD2_COUNT_MAX / 2) {
    diff_R += CUGO_LD2_COUNT_MAX;
  } else if (diff_R > CUGO_LD2_COUNT_MAX / 2) {
    diff_R -= CUGO_LD2_COUNT_MAX;
  }

  cugo_current_count_L += diff_L;
  cugo_current_count_R += diff_R;
  cugo_prev_encoder_L = cugo_current_count_L;
  cugo_prev_encoder_R = cugo_current_count_R;
}

void ld2_encoder_reset() {
  unsigned char frame[10] = { 0xFF, 0x0E, 0x01, 0x01, 0, 0, 0, 0, 0, 0 };
  ld2_write_cmd(frame);
  cugo_current_encoder_L = 0;
  cugo_prev_encoder_L = 0;
  cugo_start_count_L = 0;
  cugo_current_count_L = 0;

  cugo_current_encoder_R = 0;
  cugo_prev_encoder_R = 0;
  cugo_start_count_R = 0;
  cugo_current_count_R = 0;
}

void ld2_set_feedback(unsigned char freq_index, unsigned char kindof_data) {  //freq 0:10[hz] 1:50[hz] 2:100[hz] kindof_data 1:Mode 2:CMD_RPM 4:CurrentRPM 8:AveCurrentRPM 128:EncorderData
  unsigned char frame[10] = { 0xFF, 0x0F, freq_index, kindof_data, 0, 0, 0, 0, 0, 0 };
  cugo_ld2_feedback_hz = cugo_ld2_index_tofreq[freq_index];
  cugo_ld2_feedback_dutation = 1000 / cugo_ld2_feedback_hz;
  ld2_write_cmd(frame);
}

void ld2_set_control_mode(unsigned char mode) {  //mode 0x00:RC_mode 0x01:CMD_Mode
  unsigned char frame[10] = { 0xFF, 0x00, mode, 0, 0, 0, 0, 0, 0, 0 };
  cugo_runmode = mode;
  ld2_write_cmd(frame);
}

void cugo_reset() {
  watchdog_reboot(0, 0, 0);
  while (true)
    ;
}

void change_cmd_mode() {
  //{start_byte, CMD_ID, mode, reserved, reset, reset, reserved, reserved, reserved, reserved}
  unsigned char frame[10] = { 0xFF, 0x00, 0x01, 0, 0x01, 0x01, 0, 0, 0, 0 };
  //unsigned char frame[10] = { 0xFF, 0x00, 0x01, 0, 0x00, 0x00, 0, 0, 0, 0 };
  ld2_write_cmd(frame);
}

void return_all_data() {  //すべてのデータを返すように指示
  unsigned char frame[10] = { 0xFF, 0x0F, 0x00, 0xFF, 0, 0, 0, 0, 0, 0 };
  ld2_write_cmd(frame);
}
