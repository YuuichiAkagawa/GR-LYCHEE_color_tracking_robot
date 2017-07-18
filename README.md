# Color tracking robot with GR-LYCHEE
GR-LYCHEEプロデューサミーティング(ルネサスナイト11)で発表したGR-LYCHEEで物体追跡(色検出)ロボのプログラムです。

ベータ版のボードで開発したものなので、製品版では動かない可能性があります。販売開始後に改めて検証する予定です。

ESP32には[esp32-DisplayApp-WebServer](https://github.com/YuuichiAkagawa/esp32-DisplayApp-WebServer)を書き込んでください。

## 設定方法
- DBG_CAPTURE : カメラ画像や認識結果をSDカードに保存する機能を有効化(FEATURE_FOLLOWとは排他)
- DBG_PCMONITOR : DisplayAppでカメラ画像を表示する機能を有効化(FEATURE_ESP32MONITORとは排他)
- FEATURE_ESP32MONITOR : WiFi経由でカメラ画像を表示する機能を有効化(DBG_PCMONITORとは排他)
- FEATURE_FOLLOW : 追跡機能を有効化(DBG_CAPTUREとは排他)

## US0/US1の機能
### FEATURE_FOLLOWの場合
US0でモータ制御のOn/Offが出来ます。

### FEATURE_ESP32MONITORの場合
US1で画像送信のOn/Offが出来ます。WiFi機能自体の無効化はできません。

### DBG_CAPTUREの場合
シャッタボタンです。US0を押すとカメラ画像や認識結果をSDカードに保存します。

## LEDの機能
- LED1 : 動作中を表すLED。リセット後、初期化中の間は消灯。
- LED2 : モータ制御の有効/無効を表す。
- LED3 : WiFi経由での画像送信の有効/無効を表す。
- LED4 : 物体認識がされてると点灯。

## 認識する色の調整
main.cppのdetectColor()関数で設定する。  
Y,U,V値に対してチェックを行い、検知する色の場合はtrue,違う場合はfalseを返す。

## ビルド方法
[GR-LYCHEE用オフライン開発環境の手順](https://developer.mbed.org/users/dkato/notebook/offline-development-lychee-langja/)を参考に開発環境を構築後、以下のコマンドを実行するとビルドできます。
```
$ mbed update
$ mbed compile -m GR_LYCHEE -t GCC_ARM --profile debug
```

## その他
ラベリング処理に井村研究室のラベリングクラス(Labering.h)を利用しています。  
Copyright (c) 2010, IMURA Masataka  
http://imura-lab.org/products/labeling/


---
Copyright &copy; 2017 Yuuichi Akagawa

Licensed under the Apache License 2.0 as described in the file LICENSE.
