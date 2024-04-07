/* MIT License

Copyright (c) 2024 Mitsumine Suzu (verylowfreq)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#error This sketch should be used when USB is in OTG mode
#endif

typedef struct {
  int sensor_id;
  float x;
  float y;
  float z;
} AccelerationData_t;

typedef struct {
  int sensor_id;
  float x;
  float y;
  float z;
  float w;
} RotationData_t;

AccelerationData_t acceleration_data[16];
RotationData_t rotation_data[16];

void init_data() {
  for (int i = 0; i < 16; i++) {
    acceleration_data[i].sensor_id = -1;
    rotation_data[i].sensor_id = -1;
  }
}

unsigned long prev_time[16];
// Each sensor can send the data at 60fps.
// FIXME: Is 60fps good ?
unsigned long minimum_interval = 1000 / 60;

#include <USBCDC.h>
USBCDC USBSerial;

#include <esp_now.h>
#include <WiFi.h>


#define PACKET_ACCEL 4
#define PACKET_ROTATION_DATA 17

union float_bytes {
  uint8_t b[4];
  float f;
};

// Parse the big-endian bytes as float. NOTE: ESP32 is little-endian.
float get_float_from_4bytes(const uint8_t* buf) {
  union float_bytes val;
  val.b[0] = buf[3];
  val.b[1] = buf[2];
  val.b[2] = buf[1];
  val.b[3] = buf[0];
  return val.f;
}

void write_2bytes_from_float(float val, uint8_t* buf) {
  /* REF: https://github.com/SlimeVR/SlimeVR-Server/commit/3f6a26f7d849d1f6412709e6f7570ef98eeff4ab#diff-1761445cd329ae08306febbcfb7007d9731d748c9844e973120d8e3bdaa6579fR186
     In short: ((int16_t)buf[1] << 8 | buf[0]) / (float)(1 >> 14)
  */

  val = val * (float)(1 << 14);
  int val_int = (int)val;
  int16_t val_i16 = (int16_t)val_int;
  uint16_t val_u16 = (uint16_t)(((int)val_i16) + 32768);
  buf[0] = val_u16 & 0xff;
  buf[1] = val_u16 >> 8;
}

esp_now_peer_info_t peer;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  // Magic number in the header of the ESPNOW packet: 'S', 'L', 'V', 'R', 0x00
  if (data_len < 5 || memcmp(data, "SLVR\0", 5) != 0) {
    return;
  }

  int cur = 5;
  cur += 3;


  if (data[cur] == PACKET_ACCEL) {
    cur++;
    uint64_t packet_number = 0;
    (void)packet_number;
    cur += 8;
    float x = get_float_from_4bytes(&data[cur]);
    cur += 4;
    float y = get_float_from_4bytes(&data[cur]);
    cur += 4;
    float z = get_float_from_4bytes(&data[cur]);
    cur += 4;
    uint8_t sensor_id = data[cur++];
    AccelerationData_t data = {
      .sensor_id = sensor_id,
      .x = x,
      .y = y,
      .z = z
    };
    acceleration_data[sensor_id & 0x0f] = data;


  } else if (data[cur] == PACKET_ROTATION_DATA) {
    cur++;
    uint64_t packet_number = 0; //data[cur];
    (void)packet_number;
    cur += 8;
    uint8_t sensor_id = data[cur++];
    uint8_t data_type = data[cur++];
    (void)data_type;
    float x = get_float_from_4bytes(&data[cur]);
    cur += 4;
    float y = get_float_from_4bytes(&data[cur]);
    cur += 4;
    float z = get_float_from_4bytes(&data[cur]);
    cur += 4;
    float w = get_float_from_4bytes(&data[cur]);
    cur += 4;
    uint8_t accuracy_info = data[cur++];
    (void)accuracy_info;

    RotationData_t data = {
      .sensor_id = sensor_id,
      // WORKAROUND: Change axis to work propery. I don't know why...
      .x = z,
      .y = x,
      .z = y,
      .w = w
    };
    rotation_data[sensor_id & 0x0f] = data;
  }
}



#include "USB.h"
#include "USBHID.h"
USBHID HID;

static const uint8_t report_descriptor[] = { // 8 axis
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x00,        // Usage (Undefined)
  0xA1, 0x01,        // Collection (Application)

  // Define 4 bytes (x4 8-bits) data

  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0xFF,        //   Logical Maximum (-1)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x04,        //   Report Count (4)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

  // Define 16 bytes (x8 16-bits) data

  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x09, 0x00,        //   Usage (Undefined)
  0x15, 0x00,        //   Logical Minimum (0)
  0x26, 0xFF, 0xFF,  //   Logical Maximum (-1)
  0x75, 0x10,        //   Report Size (16)
  0x95, 0x08,        //   Report Count (8)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

  0xC0,              // End Collection
};

class CustomHIDDevice: public USBHIDDevice {
public:
  CustomHIDDevice(void){
    static bool initialized = false;
    if(!initialized){
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
  }
  
  void begin(void){
    HID.begin();
  }
    
  uint16_t _onGetDescriptor(uint8_t* buffer){
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }

  bool send(uint8_t * value, size_t len, unsigned int timeout_msec){
    return HID.SendReport(0, value, len, timeout_msec);
  }
};

CustomHIDDevice Device;

constexpr int PIN_NEOPIXEL = 35;

void panic() {
  for (int i = 0; i < 4; i++) {
    neopixelWrite(PIN_NEOPIXEL, 128, 0, 0);
    delay(500);
    neopixelWrite(PIN_NEOPIXEL, 128, 0, 0);
    delay(500);
  }
}

void setup() {
  for (int i = 0; i < 4; i++) {
    neopixelWrite(PIN_NEOPIXEL, 32,32,32);
    delay(500);
    neopixelWrite(PIN_NEOPIXEL, 0, 0, 0);
    delay(500);
  }

  init_data();

  Device.begin();

  // VID and PID must be these values.
  USB.VID(0x2FE3);
  USB.PID(0x5652);
  USB.productName("SlimeVR Dongle");
  USB.manufacturerName("MSLabo");
  USB.begin();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // FIXME: Is this realy needed for low latency ?
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    panic();
  }

  memset(&peer, 0, sizeof(peer));
  for (int i = 0; i < 6; ++i) {
    peer.peer_addr[i] = (uint8_t)0xff;
  }
  
  esp_err_t addStatus = esp_now_add_peer(&peer);
  if (addStatus != ESP_OK) {
    panic();
  }

  esp_now_register_recv_cb(OnDataRecv);
}

unsigned int minimum_interval_global_send = 1000 / 240;

void loop() {
  // Previous time of sending HID report in milliseconds.
  static int prev_send = 0;
  // Next index for searching
  static int next_search_index = 0;

  if (millis() - prev_send < minimum_interval_global_send) {
    delay(1);
    return;
  }

  if (HID.ready()) {

    // Search the prepared sensor index

    int target_index = -1;
    for (int i = 0; i < 16; i++) {
      int cur_idx = (next_search_index + i) % 16;
      if (acceleration_data[cur_idx].sensor_id >= 0 && rotation_data[cur_idx].sensor_id >= 0) {
        if (millis() - prev_time[cur_idx] < minimum_interval) {
          // To avoid too high rate sensor, minimum sending interval time for each sensor is defined.
          continue;
        } else {
          target_index = cur_idx;
          next_search_index = (cur_idx + 1) % 16;
          break;
        }
      }
    }

    if (target_index < 0) {
      return;
    }

    uint8_t device_id = 0;
    uint8_t trackerID = target_index;

    uint8_t data[20] = {
      0x00,
      // idCombination (deviceID << 4 | trackerID)
      (uint8_t)((device_id << 4) | trackerID),
      // rssi
      0x01,
      // battery
      55,
      // battery mV
      0x04, 0x05,
      // gyro, accel
      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0
    };
    write_2bytes_from_float(rotation_data[target_index].x, &data[6]);
    write_2bytes_from_float(rotation_data[target_index].y, &data[8]);
    write_2bytes_from_float(rotation_data[target_index].z, &data[10]);
    write_2bytes_from_float(rotation_data[target_index].w, &data[12]);

    write_2bytes_from_float(acceleration_data[target_index].x, &data[14]);
    write_2bytes_from_float(acceleration_data[target_index].y, &data[16]);
    write_2bytes_from_float(acceleration_data[target_index].z, &data[18]);

    rotation_data[target_index].sensor_id = -1;
    acceleration_data[target_index].sensor_id = -1;
    prev_time[target_index] = millis();

    Device.send(data, 20, 20);

    prev_send = millis();
  }
}
