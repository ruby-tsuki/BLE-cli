#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BAT_hs.h"
#include "EXOConfig.h"
#include "Wire.h"
#include "LK_command.h"
#include <esp_gatt_common_api.h>



//--------------------------timer begin-------------------------//
hw_timer_t *tim1 = NULL;
volatile bool tim1_IRQ_count = false;

void Tim1Interrupt() {  //中断服务函数
  tim1_IRQ_count = true;
}
//----------------------timer end-------------------------//


//**********************Motor begin*********************//
LK_command LK;
void gotHundred(CAN_FRAME *frame) {
  //Serial.print("Got farme: ");
  LK.parseFrame(frame);
}
CAN_FRAME txFrame;
//**********************Motor End*********************//

//**********************IMU begin*********************//
//25t wire1
#define uint16_t unsigned int
#define iic_add25 0xa6 >> 1

//95t wire0
#define iic_add95 0xa4 >> 1


typedef struct {
  int16_t GyroAxisX;
  int16_t GyroAxisY;
  int16_t GyroAxisZ;
} Gyro;

typedef struct {
  int16_t accx;
  int16_t accy;
  int16_t accz;
} acc;
typedef struct
{
  int16_t temp;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
} gy;
float scale = 16.0 / 32768.0;        //加速度
float GyroScale = 2000.0 / 32768.0;  //陀螺仪

float scale95 = 4.0 / 32768.0;        //加速度
float GyroScale95 = 2000.0 / 32768.0;  //陀螺仪

unsigned char Re_buf;
unsigned char sign = 0;
//25t
gy my_gy;
Gyro my_gyro;
acc my_acc;

//95t
gy my_gy95t;
Gyro my_gyro95t;
acc my_acc95t;
uint16_t delay_t = 0;
byte color = 0, rgb_data[3] = { 0 };
byte ready_Ok = 0;

void Exti() {
  if (!ready_Ok)
    ready_Ok = 1;  //数据更新标志
}
void iic_read(unsigned char add, unsigned char *data, unsigned char len, int lable) {
  if (lable == 25) {
    int i = 0;
    Wire1.beginTransmission(iic_add25);
    Wire1.write(add);
    Wire1.endTransmission();
    Wire1.requestFrom(iic_add25, len);
    while (Wire1.available()) {
      data[i] = Wire1.read();
      i++;
    }
  } else if (lable == 95) {
    int i = 0;
    Wire.beginTransmission(iic_add95);
    Wire.write(add);
    Wire.endTransmission();
    Wire.requestFrom(iic_add95, len);
    while (Wire.available()) {
      data[i] = Wire.read();
      i++;
    }
  } else {
    Serial.println("IMU number incorrect");
  }
}
void iic_write(char add, unsigned char data, int lable) {
  if (lable == 25) {
    Wire1.beginTransmission(iic_add25);
    Wire1.write(add);
    Wire1.write(data);
    Wire1.endTransmission();
  } else if (lable == 95) {
    Wire.beginTransmission(iic_add25);
    Wire.write(add);
    Wire.write(data);
    Wire.endTransmission();
  } else {
    Serial.println(" IMU number incorrect ");
  }
}
//**********************IMU End*********************//

//**********************Foot pressure sensor Begin*********************//
uint8_t buf[18];
int16_t sensor[9];
uint8_t temp[49];

void InsertFrame(int16_t tmp, uint8_t i) {
  temp[i * 2 - 1] = (uint8_t)(static_cast<int16_t>(tmp) >> 8);
  temp[i * 2] = (uint8_t)(static_cast<int16_t>(tmp));
}

void sensor_measurement() {
  // Read from the slave and print out
  Wire.requestFrom(0x31, 18);
  //Serial.print("\nrecv: '");//
  int i = 0;
  while (Wire.available()) {
    //Serial.print((char)Wire.read());//
    buf[i] = Wire.read();
    //Serial.print(buf[i],HEX);//
    i++;
  }

  for (int j = 0; j < 9; j++) {
    sensor[j] = (int16_t)(buf[j * 2] << 8 | buf[j * 2 + 1]);
    InsertFrame(sensor[j], 15 + j);
    //Serial.print(sensor[j]);
    //if (j < 8)
      //Serial.print(',');
  }
  //Serial.print(i);
  //Serial.println("");
}
//**********************Foot pressure sensor End*********************//

//**********************BLE Begin*********************//
#define BLEServerName "Data_Sender"
#define SERVICE_UUID "5e9e143a-5c7b-4926-a824-b1621729ebdf"
#define RX_CHARACTERISTIC_UUID "5e9e143b-5c7b-4926-a824-b1621729ebdf"
#define TX_CHARACTERISTIC_UUID "5e9e143c-5c7b-4926-a824-b1621729ebdf"

bool setZero = false;
bool connected_state = false;

esp_bd_addr_t connectedAddress; 

class MyServerCallbacks : public BLEServerCallbacks {
  void onMtuChanged(BLEServer* pServer, uint16_t mtu) {
    Serial.printf("MTU Changed to: %d\n", mtu);
  }
  void onConnect(BLEServer *pServer,esp_ble_gatts_cb_param_t* param) {
    memcpy(connectedAddress, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    pServer->updateConnParams(connectedAddress,6,6,0,500);
    connected_state = true;
  }
  void onDisconnect(BLEServer *pServer) {
    connected_state = false;
  }
};

class MyRXCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received data from client: ");
    for (uint8_t c : value) {
      Serial.print(c, HEX);
      Serial.print(" ");
    }
    Serial.println();
    if (value == "zero") {
      LK.command_95(1, &txFrame);
      CAN0.sendFrame(txFrame);
    }
  }
};

BLECharacteristic *pRXCharacteristic;
BLECharacteristic *pTXCharacteristic;
//**********************BLE End*********************//




void setup() {

  Serial.begin(115200);
  BAT_init(PIN_BAT);
  pinMode(PIN_INT1, INPUT);
  //timer
  tim1 = timerBegin(0, 80, true);  //1MHz
  timerAttachInterrupt(tim1, Tim1Interrupt, true);
  timerAlarmWrite(tim1, 0.01*1000000, true);  
  timerAlarmEnable(tim1);

  //95t
  byte td = 1;
  Wire.setClock(400000);
  Wire.setPins(PIN_SDA, PIN_SCL);
  Wire.begin();
  delay(100);
  iic_read(0x02, &td, 1, 95);
  delay(100);
  attachInterrupt(0, Exti, RISING);


  //25t
  Wire1.setClock(400000);
  Wire1.setPins(PIN_SDA2, PIN_SCL2);
  Wire1.begin();
  delay(100);
  iic_read(0x02, &td, 1, 25);
  delay(100);
  attachInterrupt(0, Exti, RISING);

  //Foot
  delay(100);
  memset(temp, 0, 49);
  temp[0] = 0xBE;
  temp[47] = 0x0D;
  temp[48] = 0x0A;

  //Motor
  CAN0.setCANPins(PIN_CANRX, PIN_CANTX);
  if (CAN0.begin(1000000)) {
    Serial.println("Init OK ...");
  } else {
    Serial.println("Init Failed ...");
  }
  Serial.println("Ready...!");
  delay(1000);
  CAN0.watchFor();
  CAN0.setCallback(0, gotHundred);
  LK.command_92(1, &txFrame);
  CAN0.sendFrame(txFrame);

  //BLE
  BLEDevice::init("HOST");
  // 设置本地MTU为517，确保在init之后、创建服务之前调用
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(517);
  if (local_mtu_ret != ESP_OK) {
    Serial.printf("Set MTU failed, error: 0x%04X\n", local_mtu_ret);
  } else {
    Serial.println("Local MTU set to 517 (may need client negotiation)");
  }

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 创建服务
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 创建特征并指定最大长度（关键修改）
  pRXCharacteristic = pService->createCharacteristic(
    RX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pRXCharacteristic->setCallbacks(new MyRXCharacteristicCallbacks());  // 添加回调
  pRXCharacteristic->addDescriptor(new BLE2902());

  pTXCharacteristic = pService->createCharacteristic(
    TX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pTXCharacteristic->addDescriptor(new BLE2902());

  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);
  pService->start();

  // 配置广告参数
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);  // 有助于提高连接速度
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->start();

  Serial.println("BLE Setting");
  delay(1000);
}

void loop() {
  if (tim1_IRQ_count) {
    sensor_measurement();
    unsigned char data[8] = { 0 };   //25
    unsigned char data3[8] = { 0 };  //95

    unsigned char data1[6] = { 0 };  //95
    unsigned char data2[6] = { 0 };  //25

    iic_read(0x0E, data2, 6, 25);
    iic_read(0x0E, data1, 6, 95);
    LK.command_94(1, &txFrame);
    CAN0.sendFrame(txFrame);

    // int64_t motorAngle = LK.getMotorAngle();
    uint32_t circleAngle = LK.getCircleAngle();

    LK.command_9C(1, &txFrame);
    CAN0.sendFrame(txFrame);
    float Iq = LK.getIq();

    my_gyro.GyroAxisX = (int16_t)((data2[0] << 8) | data2[1]);
    my_gyro.GyroAxisY = (int16_t)((data2[2] << 8) | data2[3]);
    my_gyro.GyroAxisZ = (int16_t)((data2[4] << 8) | data2[5]);

    my_gyro95t.GyroAxisX = (int16_t)((data1[1] << 8) | data1[0]);
    my_gyro95t.GyroAxisY = (int16_t)((data1[3] << 8) | data1[2]);
    my_gyro95t.GyroAxisZ = (int16_t)((data1[5] << 8) | data1[4]);

    //Serial.print("Gyro x: ");
    // Serial.print((float)my_gyro.GyroAxisX * GyroScale);
    // Serial.print(" ");
    // Serial.print("Gyro y: ");
    // Serial.print((float)my_gyro.GyroAxisY * GyroScale);
    // Serial.print(" ");
    // Serial.print("Gyro z: ");
    // Serial.println((float)my_gyro.GyroAxisZ * GyroScale);

    iic_read(0x08, data2, 6, 25);
    iic_read(0x08, data1, 6, 95);
    my_acc.accx = (int16_t)((data2[0] << 8) | data2[1]);
    my_acc.accy = (int16_t)((data2[2] << 8) | data2[3]);
    my_acc.accz = (int16_t)((data2[4] << 8) | data2[5]);

    my_acc95t.accx = (int16_t)((data1[1] << 8) | data1[0]);
    my_acc95t.accy = (int16_t)((data1[3] << 8) | data1[2]);
    my_acc95t.accz = (int16_t)((data1[5] << 8) | data1[4]);
    // Serial.print("acc x: ");
    // Serial.print((float)my_acc.accx * scale);
    // Serial.print(" ");
    // Serial.print("acc y: ");
    // Serial.print((float)my_acc.accy * scale);
    // Serial.print(" ");
    // Serial.print("acc z: ");
    // Serial.println((float)my_acc.accz * scale);


    iic_read(0x14, data, 8, 25);
    iic_read(0x14, data3, 8, 95);
    my_gy.roll = (int16_t)((data[0] << 8) | data[1]);
    my_gy.pitch = (int16_t)((data[2] << 8) | data[3]);
    my_gy.yaw = (int16_t)((data[4] << 8) | data[5]);
    my_gy.temp = (int16_t)((data[6] << 8) | data[7]);

    my_gy95t.roll = (int16_t)((data3[1] << 8) | data3[0]);
    my_gy95t.pitch = (int16_t)((data3[3] << 8) | data3[2]);
    my_gy95t.yaw = (int16_t)((data3[5] << 8) | data3[4]);
    my_gy95t.temp = (int16_t)((data3[7] << 8) | data3[6]);
    // Serial.print("roll: ");
    // Serial.print((float)my_gy.roll / 100);
    // Serial.print(",pitch: ");
    // Serial.print((float)my_gy.pitch / 100);
    // Serial.print(",yaw:");
    // Serial.print((float)my_gy.yaw / 100);
    // Serial.print(",temp:");
    // Serial.println((float)my_gy.temp / 100);

    if (connected_state) {
      if (setZero) {
        LK.command_95(1, &txFrame);
        CAN0.sendFrame(txFrame);
        setZero = false;
        Serial.println("setZero");
        delay(5000);
      }

      //Serial.println("connected!!");
      uint8_t combinedData[sizeof(Gyro) + sizeof(acc) + sizeof(gy) + sizeof(Gyro) + sizeof(acc) + sizeof(gy) + sizeof(sensor) + sizeof(circleAngle) + sizeof(Iq)];
      size_t offset = 0;
      memcpy(combinedData + offset, &my_gyro, sizeof(Gyro));
      offset += sizeof(Gyro);
      memcpy(combinedData + offset, &my_acc, sizeof(acc));
      offset += sizeof(acc);
      memcpy(combinedData + offset, &my_gy, sizeof(gy));
      offset += sizeof(gy);
      memcpy(combinedData + offset, &my_gyro95t, sizeof(Gyro));
      offset += sizeof(Gyro);
      memcpy(combinedData + offset, &my_acc95t, sizeof(acc));
      offset += sizeof(acc);
      memcpy(combinedData + offset, &my_gy95t, sizeof(gy));
      offset += sizeof(gy);
      memcpy(combinedData + offset, sensor, sizeof(sensor));
      offset += sizeof(sensor);
      memcpy(combinedData + offset, &circleAngle, sizeof(circleAngle));
      offset += sizeof(circleAngle);
      memcpy(combinedData + offset,&Iq,sizeof(Iq));
      pTXCharacteristic->setValue(combinedData, sizeof(combinedData));
      pTXCharacteristic->notify();
      tim1_IRQ_count = false;
    }
  }
  
}