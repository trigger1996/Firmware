#ifndef __RPI_CONFIG_H

#define __RPI_CONFIG_H

#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"	// 高版本的GCC有缩进要求，低版本的没有，所以在交叉编译的时候记得注释这个而仿真的时候不用注释

#include <iostream>

//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


using namespace std;
//using namespace cv;

//设备类型
#define __Device_Raspi

//变量简称，这里使用STM32的一些手法
typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
typedef float          float32;
typedef double         float64;


#define RC_CHANNEL_NUM  6

//返回值
#define SUCCESS         0
#define FAILED          1
#define NON_VERIFIED    2
#define STANDBY			3
#define SKIPPED			4

//设备状态定义
//#define SET			0
//#define NON_INITIATED	1
//#define START_FAILED	2
//#define ERROR			3

//数学相关
#define PI 3.14159265f

//串口相关
#define USER_BAUDRATE   57600


#define UDP_TARGET_RANGE 100

// 激光雷达定义
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef MIN
	#define MIN(x,y) (x)<(y)?(x):(y)
#endif

#ifndef MAX
	#define MAX(x,y) (x)>(y)?(x):(y)
#endif

#define SWAP(x, y) (x ^= y ^= x ^= y)

#define ANGLE_ALL 360
#define Rplidar_Max_Range 6000		// 最大测量距离，单位: mm

#define LidarImageSize   600
#define LidarImageWidth  LidarImageSize
#define LidarImageHeight LidarImageSize
#define LidarImageScale  0.10		// 默认: 1 / 20


/// 飞机当前状态： 飞机通过对自身判断得到的结论，传回给服务器
/// 飞机具体命令： 飞机收到服务器的命令

/* 飞机当前的状态 */
enum Copter_Status
{
	TAKING_OFF = 1,					// 正在起飞
	HOVER	   = 2,					// 悬停没事干
	__10_4_    = 3,					// 正在飞往目的地
	LANDING    = 4,					// 正在降落
	LANDED     = 5,					// 已经降落
	MANEUVER   = 6,					// 避障
	FAILSAFE   = 7,					// 故障保护

};


/* 飞机的具体命令，除了收到Go以外所有任务都不检查航点 */
enum Server_Cmd
{
	Cmd_Standby  = 0,				// 飞机处于地面上待命，电机上锁
	Cmd_Land_Now = 1,				// 飞机在当前位置降落
	Cmd_Take_Off = 2,				// 飞机从上锁状态原地升高，起飞至预定位置
	Cmd_Hover    = 3,				// 飞机在当前位置、当前高度悬停
	Cmd_Go       = 4,				// 飞机飞向预定位置
	Cmd_ARM      = 5,				// 飞机电机上锁
	Cmd_DISARM   = 6,				// 飞机电机解锁
	Cmd_Termiate = 7,				// 结束Offboard控制

};

///
/// 飞机姿态变量
//三维坐标结构体，浮点数组成，因为怕有的地方不能#include <Copter.h>
typedef struct
{
	float X;
	float Y;
	float Z;
} __Vec3f;

typedef struct
{
	int X;
	int Y;
} __Point;

typedef struct
{
	char Hour;
	char Minute;
	char Sec;
	int  mSec;

} __time;

typedef struct
{
	int Lat;
	int Lng;
	int Alt;

} __grid;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
} __AHRS;

typedef struct
{
	float X;		// x: North y: East z: Ground
	float Y;
	float Z;
} __NED;

typedef struct
{
	int Pitch;
	int Roll;
	int Yaw;

} __AHRS_Link;


#endif // __RPI_CONFIG_H
