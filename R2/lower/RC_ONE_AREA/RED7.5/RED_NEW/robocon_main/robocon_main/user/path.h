#ifndef _PATH_H_
#define _PATH_H_

#include "navigation.h"
#include "Type.h"
#include "math_algorithm.h"
#include "chassis.h"
#include "up_down_step.h"

/* 端头坐标  X,Y方向上DT35的距离 */
#define HEAD_1_X   	 48    /* 矛尖1 */
#define HEAD_1_Y     209

#define HEAD_2_X     51    /* 拳1 */
#define HEAD_2_Y     420

#define HEAD_3_X     49    /* 掌1 */
#define HEAD_3_Y     610

#define HEAD_4_X     49    /* 掌2 */
#define HEAD_4_Y     810

#define HEAD_5_X     50    /* 拳2 */
#define HEAD_5_Y     1010

#define HEAD_6_X     47
#define HEAD_6_Y     1211

#define HEAD_RAD 		-0.004 				//夹头角度

//雷达粗略进入一个范围
#define HEAD_AREA_X  272		
#define HEAD_AREA_Y  -433
#define HEAD_AREA_RAD 0.00


//起点位置（大概）
#define start_x -292
#define start_y 90
#define start_rad 1.568

/* 对接位置 */
#define DOCK_X       -390
#define DOCK_Y       -937
#define DOCK_ANGLE    -3.14   /* 转180°面朝R1 */





//从矛头架回到启动区  角度不变，面朝矛头架重试
#define Retry_Dock_X -292
#define Retry_Dock_Y 90
#define Retry_Dock_RAD 1.568





//二区取KFS角度
#define GET_KFS_RAD 1.576







/* 二区：梅林台阶和三个入口 */

/* 入口坐标（3个） */
#define ENTRY_1_X    -3120
#define ENTRY_1_Y    -2191

#define ENTRY_2_X    -1910
#define ENTRY_2_Y    -2230

#define ENTRY_3_X     -711
#define ENTRY_3_Y   -2200

//三个入口台阶在一区映射
#define ENTRY_ONE_AREA_1X -3120
#define ENTRY_ONE_AREA_1Y -975

#define ENTRY_ONE_AREA_2X -1936
#define ENTRY_ONE_AREA_2Y -991

#define ENTRY_ONE_AREA_3X -704
#define ENTRY_ONE_AREA_3Y -1099



//12号中点 x -772 -8000 1.56//-715 y -8003 	w  1.57
//11号中点 x -1973 -8058 1.55//-1925 y -8003 w 1.586
//10号中点 -3183 -7955 1.55//x -3121 y -8003 w 1.58

/* 梅林11或12台阶结束点*/  //即10号台阶中点
#define MELIN_X_11      -3383//-3120
#define MELIN_Y_11      -7955//-8103
#define MELIN_ANGLE_11  1.57f

/* 梅林10台阶结束点*/     //约10号中点右400左右
#define MELIN_X_10      -3583//-3564
#define MELIN_Y_10     -7955//-8103
#define MELIN_ANGLE_10  1.57f


//斜坡起点中点 -4190 -8580 1.54//x -4155 y -8614

/* 上斜坡起点*/    //约中点偏左30cm
#define RAMP_STAER_X_10   -4190//-4055   //更靠中间10cm  //差值506
#define RAMP_STAER_x_11   -4110//-4055     //差值864
#define RAMP_STAER_Y   -8780//-8814    //向上20cm  //差值903

/* 上斜坡终点 */
#define RAMP_END_Y       -10273//-10296  //结束前20cm
#define RAMP_END_Y_REAL  -10473//-10496  //结束

/*斜坡曲线终点 */
#define RAMP_CURVE_END_X  -2980//-2961    //差值1161
#define RAMP_CURVE_END_Y  -10713//-10736  //差值463


/* 三区 */
/* 九宫格中层三个格子 近 */
#define GRID_1_X     		 478	//455   /* 1列 */
#define GRID_1_Y     	 	 -9690 //-9731
#define GRID_2_X      	  471//472   /* 2列 */
#define GRID_2_Y       	 -10220//-10274
#define GRID_3_X     		 473//	474   /* 3列 */
#define GRID_3_Y         -10744//-10817

#define GRID_RAD  3.132   //放置KFS角度

//三区取块
//1 -2030  -10585 -3.14
//2 -1260 -10620 3.13



//启动区
// -4453 -10912 1.57//x -4373 y-10963 -3.14


//10 九宫启动
// -4246 -6824 3.12//x -4311 -6887 -3.14  



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
