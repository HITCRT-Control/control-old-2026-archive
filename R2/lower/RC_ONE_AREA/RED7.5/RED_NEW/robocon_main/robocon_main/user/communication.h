#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_
#include "Type.h"
#include "usart.h"
#include "math_algorithm.h"
#include "tim.h"


#define CODE0 115
#define CODE1 229
#define CODEReset 0

#define LED_Count 15


//红外数据处理
#define DEFAULT_QR 0 
#define COMBINE_WEAPON_COMPLETE 1 //对接完成

//所有的红外切列都不被使用
#define FIRST_COLUMN 4 //切一列 
#define SECOND_COLUMN 6 //切二列
#define THIRD_COLUMN 7 //切三列

#define ROBOT_COMBINE_COMPLETE 12 //合体完成
#define HANDOVER_KFS_COMPLETE 13 //R1传递KFS完成
#define USE_KFS_QR 14 //放置KFS
#define START_HANDOVER_KFS 15 //R1开始给R2传递KFS
#define R2_LIFT 20 //R2站立
#define STOP_R2_ACTION 21 //强制打断R2动作

//灯带TIM
void WS2812_SET(uint8_t index,uint8_t r,uint8_t g,uint8_t b);
void WS2812_AllSet(uint8_t r,uint8_t g,uint8_t b);
void WS2812_Update(void);

//串口
void Vision_Data_Deal(vision_Data_t *p_vision_data);
void usart_inner_send(void);
void usart_inner_receive(void);
void usart_all_ctrl(void);
void usart_all_ctrl_send(void);
float GET_YAW(float gyro_yaw,float radar_yaw,float gyro_w);
void choose_pid_yaw(void);
void vision_send_task(void);
#endif
