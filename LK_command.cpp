#include "LK_command.h"


void LK_command::parseFrame(CAN_FRAME *frame){
  switch(frame->data.byte[0]){
        case 0xA1:
        case 0xA2:
        case 0xA3:
            this->temperature = (int8_t)frame->data.byte[1];
            this->iq = (int16_t)(frame->data.byte[3] << 8 | frame->data.byte[2]);
            this->speed = (int16_t)(frame->data.byte[5] << 8 | frame->data.byte[4]);
            this->encode = (uint16_t)(frame->data.byte[7] << 8 | frame->data.byte[6]);            
            // Serial.print("Iq: "); Serial.print(this->iq * 0.16f);
            // Serial.print(" Speed: "); Serial.println(this->speed);
            
            break; // 添加break

        case 0x33:
            this->accel = (int32_t)(frame->data.byte[7] << 24 | 
                            frame->data.byte[6] << 16 |
                            frame->data.byte[5] << 8  |
                            frame->data.byte[4]);
            // Serial.print("Accel: "), Serial.println(this->accel);
            break;

        case 0x92:
            this->motorAngle = (int64_t)frame->data.byte[7] << 56 |
                        (int64_t)frame->data.byte[6] << 48 |
                        (int64_t)frame->data.byte[5] << 40 |
                        (int64_t)frame->data.byte[4] << 32 |
                        (int64_t)frame->data.byte[3] << 24 |
                        (int64_t)frame->data.byte[2] << 16 |
                        (int64_t)frame->data.byte[1] << 8  |
                        (int64_t)frame->data.byte[0];
            
            Serial.print("Angle: "); 
            Serial.println(this->motorAngle / 1000.0f);
            
            break;

        case 0x94:
            this->circleAngle = (uint32_t)(frame->data.byte[7] << 24 | 
                                   frame->data.byte[6] << 16 |
                                   frame->data.byte[5] << 8  |
                                   frame->data.byte[4]);
            
            // Serial.print("CircleAngle: ");
            // Serial.println(this->circleAngle / 100.0f);
            
            break;

        case 0x95:
            //setZero = true;
            break;

        case 0x9A:
            this->status1_temperature = frame->data.byte[1];
            this->status1_voltage = (uint16_t)(frame->data.byte[4] << 8 | frame->data.byte[3]);
            this->status1_error = frame->data.byte[7]; // 修正变量名
            break;

        case 0x9C:
            this->temperature = (int8_t)frame->data.byte[1];
            this->iq = ((frame->data.byte[3]) << 8) | (frame->data.byte[2]);
            this->speed = (int16_t)(frame->data.byte[5] << 8 | frame->data.byte[4]);
            this->encode = (uint16_t)(frame->data.byte[7] << 8 | frame->data.byte[6]);            
            // Serial.print("IQ: "); Serial.print(this->iq);
            // Serial.print(" Speed: "); Serial.println(this->speed);
            
            break;
}
}

void LK_command::command_A1(uint8_t id,CAN_FRAME *frame,int16_t current)
{
  frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
  int32_t inputMin = -33;
  int32_t inputMax = 33;
  int32_t outputMin = -2048;
  int32_t outputMax = 2048;
  int32_t mappedValue = (current - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
	frame->data.uint8[0] = 0xA1;
	frame->data.uint8[4] = mappedValue & 0xFF;
	frame->data.uint8[5] = (mappedValue >> 8) & 0xFF;
}

void LK_command::command_A2(uint8_t id,CAN_FRAME *frame,int32_t speed){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
  int32_t temp = speed / 0.01;
	frame->data.uint8[0] = 0xA2;
	frame->data.uint8[4] = temp & 0xFF;
	frame->data.uint8[5] = (temp >> 8) & 0xFF;
	frame->data.uint8[6] = (temp >> 16) & 0xFF;
	frame->data.uint8[7] = (temp >> 24) & 0xFF;
}

void LK_command::command_A3(uint8_t id,CAN_FRAME *frame,int32_t angle){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
  int32_t temp = angle / 0.01; 
	frame->data.uint8[0] = 0xA3;
	frame->data.uint8[4] = temp & 0xFF;
	frame->data.uint8[5] = (temp >> 8) & 0xFF;
	frame->data.uint8[6] = (temp >> 16) & 0xFF;
	frame->data.uint8[7] = (temp >> 24) & 0xFF;
}

void LK_command::command_92(uint8_t id,CAN_FRAME *frame){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
	frame->data.uint8[0] = 0x92;
}

void LK_command::command_94(uint8_t id,CAN_FRAME *frame){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
	frame->data.uint8[0] = 0x94;
}

void LK_command::command_95(uint8_t id,CAN_FRAME *frame){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
	frame->data.uint8[0] = 0x95;
}

void LK_command::command_9A(uint8_t id,CAN_FRAME *frame){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
	frame->data.uint8[0] = 0x9A;
}

void LK_command::command_9C(uint8_t id,CAN_FRAME *frame){
	frame->id = 0x140 + id; 
	frame->rtr = 0;
	frame->extended = false;
	frame->length = 8;
	for(int i = 0;i < frame->length;i++){
		frame->data.uint8[i] = 0x00;
	}
	frame->data.uint8[0] = 0x9C;
}