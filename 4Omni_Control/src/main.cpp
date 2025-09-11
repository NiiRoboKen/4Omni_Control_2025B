#include <CAN.h>
#include <cmath>

const int TX_PIN = 5;
const int RX_PIN = 4;
const double MIN_CURRENT = -20.0;
const double MAX_CURRENT = 20.0;
const int16_t MIN_SENDNUM = -16384;
const int16_t MAX_SENDNUM = 16384;

int16_t format_send_data(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
  double proportion = (x - in_min) / (in_max - in_min);
  double out_base = (double)(out_max - out_min);
  return out_min + out_base * proportion;
}
std::pair<int8_t, int8_t> split_data(int16_t formatted_data) {
  int8_t first_data = (formatted_data >> 8) & 0xFF;
  int8_t second_data = formatted_data & 0xFF;
  return {first_data, second_data};
}
int16_t unit_data(int8_t first, int8_t second) {
  return first << 8 | second;
}
class Packet{
  private:
    int id;
    int8_t buf[8];
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
    int& Id() {
      return id;
    }
    void Send() {
      CAN.beginPacket(id);
      for(int8_t data : buf) CAN.write(data);
      CAN.endPacket();
    }
};
class RoboMasMotor{
  private:
    int id = 0;
    Packet FeedBack = Packet(0x200);
  public:
    RoboMasMotor(int set_id) {
      id = set_id;
      FeedBack.Id() += id;
    }
    int Id() {
      return id;
    }
    std::pair<int, int> SendBufNum() {
      int first = (id - 1) * 2;
      int second = (id - 1) * 2 + 1;
      return {first, second};
    }
    std::pair<int8_t, int8_t> SendBufByte(double speed_percentage) {
      double proportion = speed_percentage / 100.0;
      return split_data(format_send_data(20 * proportion, MIN_CURRENT, MAX_CURRENT, MIN_SENDNUM, MAX_SENDNUM));
    }
    void SetFeedBack(Packet feedback) {
      if(FeedBack.Id() == feedback.Id()) FeedBack = feedback;
    }
    int Angle() {
      return unit_data(FeedBack.At(0), FeedBack.At(1));
    }
    int Rpm() {
      return unit_data(FeedBack.At(2), FeedBack.At(3));
    }
    int Current() {
      return unit_data(FeedBack.At(4), FeedBack.At(5));
    }
    int Temp() {
      return FeedBack.At(6);
    }
};
class Omnix4 {
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
  public:
    Omnix4() {}
    void SendPacket() {
      TxBuf.Send();
    }
    void Shift(int x, int y, double max_speed_percentage) { // 座標から距離を求めそれベクトルから角度を求め45度分引いてベクトルを求める
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
    void Turn(double speed_percentage) {
      MotorSpeedChange(FrontLeftOmni, speed_percentage);
      MotorSpeedChange(BackLeftOmni, speed_percentage);
      MotorSpeedChange(BackRightOmni, speed_percentage);
      MotorSpeedChange(FrontRightOmni, speed_percentage);
    }
    void TestMove(double x) {
      MotorSpeedChange(FrontLeftOmni, x);
      MotorSpeedChange(BackRightOmni, 0 - x);
    }
    void SetFeedBack(Packet FeedBack) {
      FrontLeftOmni.SetFeedBack(FeedBack);
      BackLeftOmni.SetFeedBack(FeedBack);
      BackLeftOmni.SetFeedBack(FeedBack);
      FrontRightOmni.SetFeedBack(FeedBack);
    }
};

Omnix4 TestOmni = Omnix4();

void receive_callback(int packetSize) {
  int read_count = 0;
  Packet FeedBack = Packet(CAN.packetId());
  while(CAN.available()) {
    FeedBack.At(read_count);
    read_count++;
  }

}

void setup() {
  CAN.setPins(RX_PIN, TX_PIN);
  CAN.begin(1000E3);
  CAN.onReceive(receive_callback);
}

void loop() {

}

