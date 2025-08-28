# 4Omni_Control_2025B
2025年Bチームのロボマスを用いた4輪オムニの制御コード
# クラスとメソッド
## Packet Class
CANで送受信するパケットのクラスです。
```cpp
Packet hoge = Packet(PacketID);
hoge.Init(); // パケットの初期化
hoge.Id(); // パケットのID
hoge.At(Num); // パケットのNum番目のデータ(int8_t型)
hoge.Send(); // パケットを送信
```
## RoboMasMortor Class
ロボマスを制御するクラスです。
```cpp
RoboMasMortor huga = RoboMasMortor(MortorID);
huga.Id(); // モーターID
huga.SendBufNum(); // 送信パケット内での番号のpair
huga.SendBufByte(SpeedPercentage); // 速さを指定した時の送信データ内容のpair
```
## Omnix4 Class
4輪オムニの制御クラスです。
```cpp
Omnix4 hogehuga = Omnix4();
hogehuga.SendPacket() // データを送信
hogehuga.Shift(x, y, MaxSpeedPercentage) // 平行移動
hogehuga.Turn(SpeedPercentage) // 旋回
hogehuga.TestMove(x) // テスト用。左前輪と右後輪のみをx%出力で動かします。
```
