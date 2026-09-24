#ifndef _PATH_H_
#define _PATH_H_

#include "navigation.h"
#include "Type.h"
#include "math_algorithm.h"
#include "chassis.h"
#include "up_down_step.h"

/* 端头坐标  X,Y方向上DT35的距离 */
#define HEAD_1_X   	 56    /* 矛尖1 */
#define HEAD_1_Y     215

#define HEAD_2_X     55    /* 拳1 */
#define HEAD_2_Y     418

#define HEAD_3_X     55   /* 掌1 */
#define HEAD_3_Y     614

#define HEAD_4_X     56    /* 掌2 */
#define HEAD_4_Y     817

#define HEAD_5_X     55    /* 拳2 */
#define HEAD_5_Y     1009

#define HEAD_6_X     56
#define HEAD_6_Y     1202

#define HEAD_RAD 		-0.014 				//夹头角度
//雷达粗略进入一个范围
#define HEAD_AREA_X  -117		
#define HEAD_AREA_Y  -382
#define HEAD_AREA_RAD -0.014


//起点位置（大概）
#define start_x 294
#define start_y 117
#define start_rad -1.573

/* 对接位置 */
#define DOCK_X       313
#define DOCK_Y       -580
#define DOCK_ANGLE    3.12   /* 转180°面朝R1 */







//从矛头架回到启动区  角度不变，面朝矛头架重试
#define Retry_Dock_X 294
#define Retry_Dock_Y 117
#define Retry_Dock_RAD -1.573





//二区取KFS角度
#define GET_KFS_RAD -1.57







/* 二区：梅林台阶和三个入口 */

/* 入口坐标（3个） */
#define ENTRY_1_X    3096
#define ENTRY_1_Y    -2155

#define ENTRY_2_X    1894
#define ENTRY_2_Y    -2215

#define ENTRY_3_X     696
#define ENTRY_3_Y    -2146

//三个入口台阶在一区映射
#define ENTRY_ONE_AREA_1X 3072
#define ENTRY_ONE_AREA_1Y -1022

#define ENTRY_ONE_AREA_2X 1876
#define ENTRY_ONE_AREA_2Y -975

#define ENTRY_ONE_AREA_3X 696
#define ENTRY_ONE_AREA_3Y -1041



//12号中点 x 718 y -7950 w -1.57
//11号中点 x 1911 y -8001 w -1.57
//10号中点 x 3108 y -7978 1 -1.57

/* 梅林11或12台阶结束点*/  //即10号台阶中点
#define MELIN_X_11      3308
#define MELIN_Y_11      -7978
#define MELIN_ANGLE_11  -1.57f

/* 梅林10台阶结束点*/     //约10号中点右400左右
#define MELIN_X_10      3508
#define MELIN_Y_10     -7978
#define MELIN_ANGLE_10  -1.57f


//斜坡起点中点 x 4150 y -8550

/* 上斜坡起点*/    //约中点偏左30cm
#define RAMP_STAER_X_10 4150   //更靠中间10cm
#define RAMP_STAER_x_11 4080
#define RAMP_STAER_Y -8750    //向上20cm

/* 上斜坡终点 */
#define RAMP_END_Y       -10280  //结束前20cm
#define RAMP_END_Y_REAL  -10480  //结束

//chongshi  4320 -10980

/*斜坡曲线终点 */
#define RAMP_CURVE_END_X  2980
#define RAMP_CURVE_END_Y  -10720


/* 三区 */
/* 九宫格中层三个格子 近 */
#define GRID_1_X     		 	-513   /* 1列 */
#define GRID_1_Y     	 	 -9627
#define GRID_2_X      	  -510   /* 2列 */
#define GRID_2_Y       	 -10179
#define GRID_3_X     		 -510   /* 3列 */
#define GRID_3_Y         -10721

#define GRID_RAD  -3.136   //放置KFS角度


//九宫藏宝
//三区两个取KFS的点位
#define KFS_1_X 1925
#define KFS_1_Y -10560
#define KFS_1_W -3.13

#define KFS_2_X 1187
#define KFS_2_Y -10562
#define KFS_2_W -3.13

///* 九宫格前曲线结束点位 */
//#define CURVE_END_X       -600
//#define CURVE_END_Y       -10180.f
//#define CURVE_END_RAD     3.133f


void path_point_choose(ST_Nav *p_nav);
void Navigate_Task(void);
void all_path_logic(void);
void path_1(void);
void path_2(void);
void path_3(void);
void choose_action_arm(void);
void start_key(void);
void choose_point_only(void);
void path_4(void);
void choose_action_arm_repeat(void);
#endif
