# RT USB IMU driver

## Description
強化学習のデプロイ時に使用するための簡易版です。
以下のフォーマットで2000000bpsのASCIIデータが送られてくることを想定しています。

```
qx,qy,qz,qw,gx,gy,gz\n
```
