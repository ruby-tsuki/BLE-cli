#include "pgmspace.h"
#ifndef _LK_command_
#define _LK_command_
#include <esp32_can.h>
#include <Arduino.h>


class LK_command
{
  private:
    int8_t temperature = 0;
    int16_t iq = 0;
    int16_t speed = 0;
    uint16_t encode = 0;
    int32_t accel = 0;
    int64_t motorAngle = 0;
    uint32_t circleAngle = 0;
    bool setZero = false;
    int8_t status1_temperature = 0;
    uint16_t status1_voltage = 0;
    uint8_t status1_error = 0; // 修正拼写错误
    bool debugMode = false;
  public:
    void command_A1(uint8_t id,CAN_FRAME *frame,int16_t current);//转矩闭环控制——iq -33~33
    void command_A2(uint8_t id,CAN_FRAME *frame,int32_t speed);  //速度闭环控制
    void command_A3(uint8_t id,CAN_FRAME *frame,int32_t angle);  //多圈位置
    void command_92(uint8_t id,CAN_FRAME *frame);//读取多圈角度
    void command_94(uint8_t id,CAN_FRAME *frame);//读取单圈角度
    void command_95(uint8_t id,CAN_FRAME *frame);//设置当前角度（默认setzero）
    void command_9A(uint8_t id,CAN_FRAME *frame);//读取状态1：温度、母线电压、母线电流、电机状态、错误标志
    void command_9C(uint8_t id,CAN_FRAME *frame);//读取状态2：温度、转矩电流、转速、编码器值    
    void parseFrame(CAN_FRAME* frame);
    
    int64_t getMotorAngle() const { return motorAngle / 1000.0f; }
    uint32_t getCircleAngle() const { return circleAngle/ 1000.0f; }
    float getIq() const { return iq * 66.0f / 4096.0f; }

};























#endif