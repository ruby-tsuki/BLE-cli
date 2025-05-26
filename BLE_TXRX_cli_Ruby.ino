#include <BLEDevice.h>
#include "HardwareSerial.h"
#include <Arduino.h>
#include <string>  // 保留，若使用 std::string 需包含（当前代码使用 Arduino String，可移除，但无害）
#include <esp_gatt_common_api.h>
// 远程服务 UUID（与服务端一致）
static BLEUUID serviceUUID("5e9e143a-5c7b-4926-a824-b1621729ebdf");
// 客户端接收通知的特征 UUID（对应服务端的 TX 特征）
static BLEUUID RX_CLIENT_UUID("5e9e143c-5c7b-4926-a824-b1621729ebdf");
// 客户端发送数据的特征 UUID（对应服务端的 RX 特征，服务端接收数据的特征）
static BLEUUID TX_SERVER_UUID("5e9e143b-5c7b-4926-a824-b1621729ebdf");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRXRemoteCharacteristic;  // 客户端接收通知的特征
static BLERemoteCharacteristic* pTXRemoteCharacteristic;  // 客户端发送数据的特征（服务端的 RX 特征）
static BLEAdvertisedDevice* myDevice;

// 通知回调（解析服务端发送的数据）
// 通知回调（按服务端实际数据结构解析）

float GyroScale = 2000.0 / 32768.0;
float scale = 16.0 / 32768.0;

float GyroScale95 = 2000.0 / 32768.0;
float scale95 = 4.0 / 32768.0;

int32_t counter = 0;
uint32_t start = millis();

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  // Serial0.print("Notify callback for characteristic ");
  // Serial0.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  // Serial0.print(" of data length ");
  // Serial0.println(length);
  // Serial0.println("Parsed data:");
  // counter++;
  // if (counter == 100) {
  //   uint32_t end = millis();
  //   Serial0.println(end - start);
  //   counter = 0;
  //   delay(1000);
  //   start = millis();

  // }
  static char printBuffer[512];
  size_t offset = 0;
  size_t printOffset = 0;

  // 解析 25t （3个 int16_t，共 6 字节）
  struct {
    int16_t x, y, z;
  } gyro;
  gyro.x = (int16_t)(pData[offset]) | (int16_t)(pData[offset + 1]) << 8;
  gyro.y = (int16_t)(pData[offset + 2]) | (int16_t)(pData[offset + 3]) << 8;
  gyro.z = (int16_t)(pData[offset + 4]) | (int16_t)(pData[offset + 5]) << 8;
  //Serial0.printf("Gyro: X=%f, Y=%f, Z=%f\n", (float)gyro.x * GyroScale, (float)gyro.y * GyroScale, (float)gyro.z * GyroScale);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "Gyro: X=%.6f, Y=%.6f, Z=%.6f\n",
                          (float)gyro.x * GyroScale, (float)gyro.y * GyroScale, (float)gyro.z * GyroScale);
  offset += sizeof(int16_t) * 3;  // sizeof(Gyro) 等于 6（3个 int16_t）

  struct {
    int16_t ax, ay, az;
  } acc;
  acc.ax = (int16_t)(pData[offset]) | (int16_t)(pData[offset + 1]) << 8;
  acc.ay = (int16_t)(pData[offset + 2]) | (int16_t)(pData[offset + 3]) << 8;
  acc.az = (int16_t)(pData[offset + 4]) | (int16_t)(pData[offset + 5]) << 8;
  //Serial0.printf("acc: X=%f, Y=%f, Z=%f\n", (float)acc.ax * scale, (float)acc.ay * scale, (float)acc.az * scale);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "acc: X=%.6f, Y=%.6f, Z=%.6f\n",
                          (float)acc.ax * scale, (float)acc.ay * scale, (float)acc.az * scale);
  offset += sizeof(int16_t) * 3;

  struct {
    int16_t roll, pitch, yaw, temp;
  } gy;
  gy.temp = (int16_t)(pData[offset]) | (int16_t)(pData[offset + 1]) << 8;
  gy.roll = (int16_t)(pData[offset + 2]) | (int16_t)(pData[offset + 3]) << 8;
  gy.pitch = (int16_t)(pData[offset + 4]) | (int16_t)(pData[offset + 5]) << 8;
  gy.yaw = (int16_t)(pData[offset + 6]) | (int16_t)(pData[offset + 7]) << 8;
  //Serial0.printf("gy: roll=%f, pitch=%f, yam=%f\n", (float)gy.roll / 100, (float)gy.pitch / 100, (float)gy.yaw / 100);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "gy: roll=%.2f, pitch=%.2f, yaw=%.2f\n",
                          (float)gy.roll / 100, (float)gy.pitch / 100, (float)gy.yaw / 100);
  offset += sizeof(int16_t) * 4;

  //解析95t
  struct {
    int16_t x, y, z;
  } gyro95;
  gyro95.x = (int16_t)(pData[offset]) | (int16_t)(pData[offset + 1]) << 8;
  gyro95.y = (int16_t)(pData[offset + 2]) | (int16_t)(pData[offset + 3]) << 8;
  gyro95.z = (int16_t)(pData[offset + 4]) | (int16_t)(pData[offset + 5]) << 8;
  //Serial0.printf("Gyro95: X=%f, Y=%f, Z=%f\n", (float)gyro95.x * GyroScale, (float)gyro95.y * GyroScale, (float)gyro95.z * GyroScale);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "Gyro95: X=%.6f, Y=%.6f, Z=%.6f\n",
                          (float)gyro95.x * GyroScale95, (float)gyro95.y * GyroScale95, (float)gyro95.z * GyroScale95);
  offset += sizeof(int16_t) * 3;  // sizeof(Gyro) 等于 6（3个 int16_t）

  struct {
    int16_t ax, ay, az;
  } acc95;
  acc95.ax = (int16_t)(pData[offset]) | (int16_t)(pData[offset + 1]) << 8;
  acc95.ay = (int16_t)(pData[offset + 2]) | (int16_t)(pData[offset + 3]) << 8;
  acc95.az = (int16_t)(pData[offset + 4]) | (int16_t)(pData[offset + 5]) << 8;
  //Serial0.printf("acc95: X=%f, Y=%f, Z=%f\n", (float)acc95.ax * scale, (float)acc95.ay * scale, (float)acc95.az * scale);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "acc95: X=%.6f, Y=%.6f, Z=%.6f\n",
                          (float)acc95.ax * scale95, (float)acc95.ay * scale95, (float)acc95.az * scale95);
  offset += sizeof(int16_t) * 3;

  struct {
    int16_t roll, pitch, yaw, temp;
  } gy95;
  gy95.temp = (int16_t)(pData[offset]) | (int16_t)(pData[offset + 1]) << 8;
  gy95.roll = (int16_t)(pData[offset + 2]) | (int16_t)(pData[offset + 3]) << 8;
  gy95.pitch = (int16_t)(pData[offset + 4]) | (int16_t)(pData[offset + 5]) << 8;
  gy95.yaw = (int16_t)(pData[offset + 6]) | (int16_t)(pData[offset + 7]) << 8;
  //Serial0.printf("gy95: roll=%f, pitch=%f, yam=%f\n", (float)gy95.roll / 100, (float)gy95.pitch / 100, (float)gy95.yaw / 100);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "gy95: roll=%.2f, pitch=%.2f, yaw=%.2f\n",
                          (float)gy95.roll / 100, (float)gy95.pitch / 100, (float)gy95.yaw / 100);
  offset += sizeof(int16_t) * 4;

  // 解析 sensor 数组（9个 int16_t，共 18 字节）
  int16_t sensor[9];
  memcpy(sensor, pData + offset, sizeof(int16_t) * 9);
  //Serial0.print("Sensor (9 values): ");
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "Sensor (9 values): ");
  for (int i = 0; i < 9; i++) {
    // Serial0.print(sensor[i]);
    // if (i < 8) Serial0.print(", ");
    printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                            "%d%s", sensor[i], i < 8 ? ", " : "\n");
  }
  //Serial0.println();
  offset += sizeof(int16_t) * 9;  // 18 字节

  // // 解析 motorAngle（int64_t，8 字节，小端模式）
  // int64_t motorAngle;
  // // 小端模式：低字节在前，高字节在后
  // motorAngle = (int64_t)(pData[offset]) | (int64_t)(pData[offset + 1]) << 8 | (int64_t)(pData[offset + 2]) << 16 | (int64_t)(pData[offset + 3]) << 24 | (int64_t)(pData[offset + 4]) << 32 | (int64_t)(pData[offset + 5]) << 40 | (int64_t)(pData[offset + 6]) << 48 | (int64_t)(pData[offset + 7]) << 56;
  // // Serial0.print("MotorAngle: ");
  // // Serial0.println(motorAngle / 100.0f);
  // printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
  //                         "MotorAngle: %.2f\n", motorAngle / 100.0f);
  // offset += sizeof(int64_t);  // 8 字节

  // 解析 circleAngle（uint32_t，4 字节，小端模式）
  uint32_t circleAngle;
  circleAngle = (uint32_t)(pData[offset]) | (uint32_t)(pData[offset + 1]) << 8 | (uint32_t)(pData[offset + 2]) << 16 | (uint32_t)(pData[offset + 3]) << 24;
  //Serial0.printf("CircleAngle: %u\n", circleAngle);
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "CircleAngle: %u\n", circleAngle);
  offset += sizeof(uint32_t);  // 4 字节

  //Iq
  float iqValue;
  uint32_t iqRaw;
  iqRaw = (uint32_t)(pData[offset]) | (uint32_t)(pData[offset + 1]) << 8 | (uint32_t)(pData[offset + 2]) << 16 | (uint32_t)(pData[offset + 3]) << 24;
  memcpy(&iqValue, &iqRaw, sizeof(float));
  printOffset += snprintf(printBuffer + printOffset, sizeof(printBuffer) - printOffset,
                          "Iq: %.4f\n", iqValue);
  offset += sizeof(float);

  Serial0.println(printBuffer);
}


// 客户端连接回调
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    BLEAddress addr = myDevice->getAddress();
    esp_err_t ret = esp_ble_gap_set_prefer_conn_params(*addr.getNative(), 6, 6, 0, 500);
    if (ret == ESP_OK) {
      Serial0.println("Connection parameters set successfully");
    } else {
      Serial0.printf("Set parameters failed, error code: 0x%x\n", ret);
    }
    connected = true;
    Serial0.println("Connected to the server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    doConnect = false;
    doScan = true;
    Serial0.println("Disconnected from the server, starting scan again");
  }
};



// 连接到服务端
bool connectToServer() {
  Serial0.print("Forming a connection to ");
  Serial0.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial0.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial0.println("Connection failed");
    delete pClient;
    return false;
  }
  pClient->setMTU(517);

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (!pRemoteService) {
    Serial0.println("Failed to find service UUID");
    pClient->disconnect();
    return false;
  }

  // 获取客户端接收通知的特征（服务端的 TX 特征）
  pRXRemoteCharacteristic = pRemoteService->getCharacteristic(RX_CLIENT_UUID);
  if (!pRXRemoteCharacteristic) {
    Serial0.println("Failed to find RX characteristic");
    pClient->disconnect();
    return false;
  }

  // 获取客户端发送数据的特征（服务端的 RX 特征）
  pTXRemoteCharacteristic = pRemoteService->getCharacteristic(TX_SERVER_UUID);
  if (!pTXRemoteCharacteristic) {
    Serial0.println("Failed to find TX characteristic (server's RX)");
    pClient->disconnect();
    return false;
  }

  // 注册通知回调
  if (pRXRemoteCharacteristic->canNotify()) {
    pRXRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  return true;
}

// 设备扫描回调（添加分号）
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial0.print("BLE Advertised Device found: ");
    Serial0.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.getName() == "HOST") {  // 根据服务端名称筛选（可增加服务 UUID 检查）
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};  // 添加分号

// 发送数据到服务端（修正 writeValue 和数据长度）
void sendDataToServer(const String& data) {
  if (connected && pTXRemoteCharacteristic) {
    pTXRemoteCharacteristic->writeValue(data.c_str(), data.length());  // 正确写入方法

    Serial0.println("Data sent to server");
  } else {
    Serial0.println("Not connected or characteristic not found");
  }
}

void setup() {
  Serial0.begin(460800);
  Serial0.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(517);
  if (local_mtu_ret) {
    ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
  }
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);  //интервал сканирования 1,3 сек
  pBLEScan->setWindow(449);     //окно сканирования 449 мс
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5);  // 初始扫描 5 秒
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial0.println("Successfully connected to the BLE Server.");
      doConnect = false;
    } else {
      Serial0.println("Connection failed, retrying scan.");
      doConnect = false;
      doScan = true;
    }
  }

  if (doScan) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->start(5);  // 重新扫描
    doScan = false;
  }

  // 串口输入触发发送（使用 Arduino String）
  if (connected && Serial0.available() > 0) {
    String data = Serial0.readStringUntil('\n');
    sendDataToServer(data);
  }
}