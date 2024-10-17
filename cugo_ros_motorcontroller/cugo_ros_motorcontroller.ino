#include "CugoSDK.h"
#include <Servo.h>
#include <PacketSerial.h>

#define SERIAL_BIN_BUFF_SIZE 64
#define SERIAL_HEADER_SIZE 8
uint8_t packetBinaryBufferSerial[SERIAL_HEADER_SIZE + SERIAL_BIN_BUFF_SIZE];
PacketSerial packetSerial;

// 受信バッファ
#define RECV_HEADER_CHECKSUM_PTR 6 // ヘッダ チェックサム
#define TARGET_RPM_L_PTR 0 // 左モータ目標RPM
#define TARGET_RPM_R_PTR 4 // 右モータ目標RPM

// 送信バッファ
#define SEND_ENCODER_L_PTR 0 // 左エンコーダ回転数
#define SEND_ENCODER_R_PTR 4 // 右エンコーダ回転数

unsigned long long current_time = 0, prev_time_10ms = 0, prev_time_100ms, prev_time_1000ms; // オーバーフローしても問題ないが64bit確保

// FAIL SAFE COUNT
int COM_FAIL_COUNT = 0;

void stop_motor_immediately()
{
  cugo_rpm_direct_instructions(0, 0);
}

void check_failsafe()
{
  // 100msごとに通信の有無を確認
  // 5回連続(0.5秒)ROSからの通信が来なかった場合、直ちにロボットを停止する
  COM_FAIL_COUNT++;
  if (COM_FAIL_COUNT > 5)
  {
    stop_motor_immediately();
  }
}

void job_10ms()
{
  // nothing
}

void job_100ms()
{
  check_failsafe();
  // エンコーダカウントをSDKから取得
  ld2_get_cmd();
  change_cmd_mode();
}

void job_1000ms()
{
  //nothing
}

void write_float_to_buf(uint8_t* buf, const int TARGET, float val)
{
  uint8_t* val_ptr = reinterpret_cast<uint8_t*>(&val);
  memmove(buf + TARGET, val_ptr, sizeof(float));
}

void write_int_to_buf(uint8_t* buf, const int TARGET, int val)
{
  uint8_t* val_ptr = reinterpret_cast<uint8_t*>(&val);
  memmove(buf + TARGET, val_ptr, sizeof(int));
}

void write_bool_to_buf(uint8_t* buf, const int TARGET, bool val)
{
  uint8_t* val_ptr = reinterpret_cast<uint8_t*>(&val);
  memmove(buf + TARGET, val_ptr, sizeof(bool));
}

float read_float_from_buf(uint8_t* buf, const int TARGET)
{
  float val = *reinterpret_cast<float*>(buf + SERIAL_HEADER_SIZE + TARGET);
  return val;
}

int read_int_from_buf(uint8_t* buf, const int TARGET)
{
  int val = *reinterpret_cast<int*>(buf + SERIAL_HEADER_SIZE + TARGET);
  return val;
}

bool read_bool_from_buf(uint8_t* buf, const int TARGET)
{
  bool val = *reinterpret_cast<bool*>(buf + SERIAL_HEADER_SIZE + TARGET);
  return val;
}

uint8_t read_uint8_t_from_buf(uint8_t* buf, const int TARGET)
{
  uint8_t val = *reinterpret_cast<uint8_t*>(buf + SERIAL_HEADER_SIZE + TARGET);
  return val;
}

uint16_t read_uint16_t_from_header(uint8_t* buf, const int TARGET)
{
  if (TARGET >= SERIAL_HEADER_SIZE - 1) return 0;
  uint16_t val = *reinterpret_cast<uint16_t*>(buf + TARGET);
  return val;
}

void set_motor_cmd_binary(uint8_t* reciev_buf, int size)
{
  if (size > 0)
  {
    // 2輪の場合
    float sp_reciev_float[2];
    sp_reciev_float[0] = read_float_from_buf(reciev_buf, TARGET_RPM_L_PTR);
    sp_reciev_float[1] = read_float_from_buf(reciev_buf, TARGET_RPM_R_PTR);
    //split(reciev_str, ',', sp_reciev_str);

    int rpm_l = sp_reciev_float[0];
    if(abs(rpm_l) >= CUGO_NORMAL_MOTOR_RPM) rpm_l = rpm_l / abs(rpm_l) * CUGO_NORMAL_MOTOR_RPM;
    int rpm_r = sp_reciev_float[1];
    if(abs(rpm_r) >= CUGO_NORMAL_MOTOR_RPM) rpm_r = rpm_r / abs(rpm_r) * CUGO_NORMAL_MOTOR_RPM;
    cugo_rpm_direct_instructions(rpm_l, rpm_r);
    /*  モータに指令値を無事セットできたら、通信失敗カウンタをリセット
        毎回リセットすることで通常通信できる。
        10Hzで通信しているので、100msJOBでカウンタアップ。
    */
    COM_FAIL_COUNT = 0;
  }
  else
  {
    cugo_rpm_direct_instructions(0.0, 0.0);
  }
}

uint16_t calculate_checksum(const void* data, size_t size, size_t start = 0)
{
  uint16_t checksum = 0;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  // バイト列を2バイトずつ加算
  for (size_t i = start; i < size; i += 2)
  {
    checksum += (bytes[i] << 8) | bytes[i + 1];
  }
  // 桁あふれがあった場合は回収
  checksum = (checksum & 0xFFFF) + (checksum >> 16);
  // チェックサムを反転
  return ~checksum;
}

void create_serial_packet(uint8_t* packet, uint16_t* header, uint8_t* body)
{
  size_t offset = 0;
  memmove(packet, header, sizeof(uint8_t)*SERIAL_HEADER_SIZE);
  offset += sizeof(uint8_t) * SERIAL_HEADER_SIZE;
  memmove(packet + offset, body, sizeof(uint8_t)*SERIAL_BIN_BUFF_SIZE);
}

void onSerialPacketReceived(const uint8_t* buffer, size_t size)
{
  uint8_t tempBuffer[size];
  memcpy(tempBuffer, buffer, size);
  // バッファにたまったデータを抜き出して制御に適用
  // チェックサムの確認
  uint16_t recv_checksum = read_uint16_t_from_header(tempBuffer, RECV_HEADER_CHECKSUM_PTR);
  uint16_t calc_checksum = calculate_checksum(tempBuffer, SERIAL_HEADER_SIZE + SERIAL_BIN_BUFF_SIZE, SERIAL_HEADER_SIZE);
  if (recv_checksum != calc_checksum)
  {
    //Serial.println("Packet integrity check failed");
  }
  else
  {
    set_motor_cmd_binary(tempBuffer, size);
  }

  // 送信ボディの作成
  uint8_t send_body[SERIAL_BIN_BUFF_SIZE];
  // 送信ボディの初期化
  memset(send_body, 0, sizeof(send_body));
 
  // ボディへ送信データの書き込み
  write_float_to_buf(send_body, SEND_ENCODER_L_PTR, cugo_current_count_L);
  write_float_to_buf(send_body, SEND_ENCODER_R_PTR, cugo_current_count_R);

  // チェックサムの計算
  uint16_t checksum = calculate_checksum(send_body, SERIAL_BIN_BUFF_SIZE);
  uint16_t send_len = SERIAL_HEADER_SIZE + SERIAL_BIN_BUFF_SIZE;
  // 送信ヘッダの作成
  uint16_t localPort = 8888;
  uint16_t send_header[4] = {localPort, 8888, send_len, checksum};

  // 送信パケットの作成
  uint8_t send_packet[send_len];
  create_serial_packet(send_packet, send_header, send_body);

  packetSerial.send(send_packet, send_len);
}

void setup()
{
  //プロポでラジコンモード切替時に初期化したい場合はtrue、初期化しない場合はfalse 
  cugo_switching_reset = false;

  cugo_init();//初期設定
  packetSerial.begin(115200);
  packetSerial.setStream(&Serial);
  packetSerial.setPacketHandler(&onSerialPacketReceived);

}

void loop()
{
  current_time = micros();

  if (current_time - prev_time_10ms > 10000)
  {
    job_10ms();
    prev_time_10ms = current_time;
  }

  if (current_time - prev_time_100ms > 100000)
  {
    job_100ms();
    prev_time_100ms = current_time;
  }

  if (current_time - prev_time_1000ms > 1000000)
  {
    job_1000ms();
    prev_time_1000ms = current_time;
  }

  //// シリアル通信でコマンドを受信するとき
  packetSerial.update();
  // 受信バッファのオーバーフローチェック(optional).
  if(packetSerial.overflow())
  {
    //Serial.print("serial packet overflow!!");
  }

}
