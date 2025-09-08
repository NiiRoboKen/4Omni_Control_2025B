#include <CAN.h>
#include <cmath>

const int TX_PIN = 5;
const int RX_PIN = 4;
const double MIN_CURRENT = -20.0;
const double MAX_CURRENT = 20.0;
const int16_t MIN_SENDNUM = -16384;
const int16_t MAX_SENDNUM = 16384;

// 浮動小数点型の値を16bit整数型の値にあてはめます。
int16_t format_send_data(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
  double proportion = (x - in_min) / (in_max - in_min);
  double out_base = (double)(out_max - out_min);
  return out_min + out_base * proportion;
}
// 16bit整数型の値を8bit整数型のデータに分割しpair型で返します。
std::pair<int8_t, int8_t> split_data(int16_t formatted_data) {
  int8_t first_data = (formatted_data >> 8) & 0xFF;
  int8_t second_data = formatted_data & 0xFF;
  return {first_data, second_data};
}

// CAN通信で送信するパケットのclassです。
class Packet{
  public:
    Packet(int set_id) { 
      id = set_id;
      Init();
    }
    void Init() {
      for(int8_t& data : buf) data = 0x00;
    }
    int8_t& At(int num) {
      return buf[num];
    }
    int Id() {
      return id;
    }
    void Send() {
      CAN.beginPacket(id);
      for(int8_t data : buf) CAN.write(data);
      CAN.endPacket();
    }
  private:
    int id;
    int8_t buf[8];
};
// ロボマスモーターのclassです。
class RoboMasMotor{
  public:
    RoboMasMotor(int set_id) {
      id = set_id;
    }
    int Id() {
      return id;
    }
    // 自らのIDから送信パケット内でのデータの位置(2Byte分)を割り出しpair型で返します。
    std::pair<int, int> SendBufNum() {
      int first = (id - 1) * 2;
      int second = (id - 1) * 2 + 1;
      return {first, second};
    }
    // スピードを百分率で受け取り、対応するデータ(2Byte分)を1ByteずつPair型で返します。
    std::pair<int8_t, int8_t> SendBufByte(double speed_percentage) {
      double proportion = speed_percentage / 100.0;
      return split_data(format_send_data(20 * proportion, MIN_CURRENT, MAX_CURRENT, MIN_SENDNUM, MAX_SENDNUM));
    }
  private:
    int id;
};
// 4輪オムニのclassです。
class Omnix4 {
  public:
    Omnix4() {}
    void SendPacket() { // 下記メソッドで編集されたパケットを送信します。
      TxBuf.Send();
    }
    // コントロールスティックの座標から平行移動用にパケットを編集します。安全を鑑みて最大出力を指定してください。
    void Shift(int x, int y, double max_speed_percentage) {
      double distans = std::sqrt(x * x + y * y);
      double radian = std::acos(x / distans);
      if(y < 0) radian = 0 - radian;
      radian -= PI/4;
      double vector13 = std::cos(radian) * max_speed_percentage;
      double vector24 = std::sin(radian) * max_speed_percentage; 
      MotorSpeedChange(FrontLeftOmni, vector13);
      MotorSpeedChange(BackLeftOmni, vector24);
      MotorSpeedChange(BackRightOmni, 0 - vector13);
      MotorSpeedChange(FrontRightOmni, 0 - vector24);
    }
    void Turn(double speed_percentage) { //±100%の範囲の値を受け取って旋回用にパケットを編集します。
      MotorSpeedChange(FrontLeftOmni, speed_percentage);
      MotorSpeedChange(BackLeftOmni, speed_percentage);
      MotorSpeedChange(BackRightOmni, speed_percentage);
      MotorSpeedChange(FrontRightOmni, speed_percentage);
    }
  private:
    RoboMasMotor FrontLeftOmni = RoboMasMotor(1);
    RoboMasMotor BackLeftOmni = RoboMasMotor(2);
    RoboMasMotor BackRightOmni = RoboMasMotor(3);
    RoboMasMotor FrontRightOmni = RoboMasMotor(4);
    Packet TxBuf = Packet(0x200);
    const double MAX_CONTROLLER_INPUT = 127.0;
    void MotorSpeedChange(RoboMasMotor motor, int speed_percentage) {
      TxBuf.At(motor.SendBufNum().first) = motor.SendBufByte(speed_percentage).first;
      TxBuf.At(motor.SendBufNum().second) = motor.SendBufByte(speed_percentage).second;
    }
};

Omnix4 TestOmni = Omnix4();

void setup() {
  CAN.setPins(RX_PIN, TX_PIN);
  CAN.begin(1000E3);
}

void loop() {
}
