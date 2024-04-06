# SlimeVR USB-HID dongle for trackers over ESP-NOW

This is a USB-HID dongle code for M5AtomS3. The sensor data is received over ESP-NOW.

SlimeVR server v0.12.0 rc.1 introduced USB-HID sensor support. This code uses this feature.

**Not tested well.**

USB接続の無線ドングルのコードです。M5AtomS3で動きます。センサーからはESP-NOWでデータを受信します。

SlimeVR server v0.12.0 rc.1 ではUSB-HIDのサポートが追加されていて、それを利用しています。

十分にはテストされていません。

![Screenshot of SlimeVR server recognizing the dongle](https://github.com/verylowfreq/slimevr-hiddongle-espnow/assets/60875431/cf3b351c-161c-4827-9003-18191dc0ea14)

Video (my tweet): https://twitter.com/verylowfreq/status/1776414271632806100/

## Prerequisites / 用意するもの
 - **SlimeVR server v0.12.0 rc.1** or later
 - SlimeVR tracker over ESP-NOW
   - Use my fork. One direction transmission (sensor to dongle; no pairing)
 - M5AtomS3 (ESP32-S3)
   - Should be work with generic ESP32-S3 board (except NeoPixel on GPIO 35)
 - Arduino IDE / Board: **arduino-esp32**
   - NOT the board library provide from M5Stack

## How to build / ビルド方法

 1. Open the .ino file.
 2. Tool -> USB Mode: USB-OTG, Upload Mode: USB-OTG
 3. Verify
 4. Upload
 5. Reset M5AtomS3 manually

## Known issues / 既知の問題

 - USB unstable ?
 - Battery level, battery voltage and RSSI is fixed value.
 - Tune the communication rate over USB-HID

## Notes / メモ

 - Based on https://github.com/SlimeVR/SlimeVR-Server/pull/913/commits
 - USB VID and PID is required the following values: VID=0x2FE3 PID=0x5652
