# !WARN! this Branch doesn't pass test.
## RoboMasMortor Class
ロボマスを制御するクラスです。
```cpp
RoboMasMortor huga = RoboMasMortor(MortorID);
huga.Id(); // モーターID
huga.SendBufNum(); // 送信パケット内での番号のpair
huga.SendBufByte(SpeedPercentage); // 速さを指定した時の送信データ内容のpair
huga.SetFeedBack(FeedBackPacket); // フィードバックを反映
huga.Angle(); // 角度(生数値)
huga.Rpm(); // RPM
huga.Current(); // 電流(生数値)
huga.Temp(); // 温度
```
