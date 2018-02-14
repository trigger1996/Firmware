#ifndef __Self_Navigator_H

#define __Self_Navigator_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#include <rpi_config.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>


#include <systemlib/mavlink_log.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <termios.h>
#include <drivers/drv_hrt.h>
#include <vector>
#include <deque>

#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <uORB/topics/input_rc.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/socket_recv.h>

#include <uORB/topics/vehicle_control_mode.h>

#include <poll.h>

#include <opencv2/opencv.hpp>

#include "coordinate_conversion.h"


using namespace std;
using namespace cv;


#define SELF_NAV_RC_THRESHOLD	1750

#define GPS_ERROR_THRESHOLD		550			// 经纬度乘以10^7以后的允许误差
#define ALT_ERROR_THRESHOLD		0.20f		// 高度误差允许值，单位m
#define HOVER_TIME_SIMPLE		5			// 单位s，Simple Task中悬停的时间，如果用hrt_absolute_time(), 则0000000 -> 10s
#define TIME_OVERFLOW_THRESHOLD	65			// 65s，hrt_absolute_time()这个东西有时候会读出来负数，不准，这边加个保护

extern "C" __EXPORT int self_navigator_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int self_navigator_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
void self_nav_usage(const char *reason);




class __self_nav
{
public:

	__self_nav();


	// 收到数据
	struct vehicle_command_ack_s _ack;

	/// 基本模式选择
	bool is_gps_online;
	bool is_lidar_online;
	//bool is_datalink_online;

	int Init();

	int update_HomePos();
	int update_VehicleStatus();
	int update_User_RC(int nms);
	int update_User_Status(int nms);
	int update_Subscribe();
	int update_CmdACK(int ms);
	int update_HeartBeat();

	int select_FlightMode();
	int set_Offboard_Mode(bool setOffboard);
	int set_ARM_DISARM(bool setARM);
	int set_Position(float x, float y, float z);		// 注意：这里是NED坐标，但是z取了相反数，所以以正值表示天上
	int set_Position(float x, float y, float z, float vx, float vy);
	int Takeoff(float alt);
	int Land();

	/// 路径规划函数
	int get_Tgt_NED(__NED tgt);
	int get_Current_NED(__NED current);
	int Flight_Simple();

	/// 订阅的数据
	// 电池状态订阅
	struct battery_status_s bat;
	// 遥控器输入订阅
	struct input_rc_s rc_in;
	//位置信息订阅
	struct vehicle_local_position_s lcl_pos;					// 本地位置
	struct vehicle_global_position_s gbl_pos, gbl_pos_last;		// 全球位置
	// 姿态信息订阅
	struct vehicle_attitude_s sensor;							// 飞机姿态
	// 飞行器状态
	struct vehicle_status_s vehicle_stat;
	// 起飞位置订阅
	struct home_position_s home_pos;
	// Socket命令订阅
	struct socket_recv_s skt_recv;

	// 回传
	int bat_ret;
	int rc_in_ret;
	int gbl_pos_ret;
	int lcl_pos_ret;
	int sensor_ret;
	int vehicle_stat_ret;
	int home_pos_ret;
	int skt_recv_ret;


	/// 发布的数据
	struct vehicle_command_s _command;
	struct position_setpoint_triplet_s _pos_sp_triplet;
	struct offboard_control_mode_s _ocm;

	struct vehicle_control_mode_s _ctrl_mode;

//private:

	/// 调试的时候可以屏蔽private

	/// uORB相关
	/// 数据订阅
	int bat_sub_fd;
	int rc_in_sub_fd;
	int gbl_pos_sub_fd;
	int lcl_pos_sub_fd;
	int sensor_sub_fd;
	int vehicle_stat_sub_fd;
	int home_pos_sub_fd;
	int skt_recv_sub_fd;

	int vehicle_command_ack_sub;		// vehicle_command_ack是发送命令vehicle_command的返回信息,可以查看命令是否执行成功

	/// 数据发布
	orb_advert_t vehicle_command_pub;
	orb_advert_t pos_sp_triplet_pub;
	orb_advert_t offboard_pub;

	orb_advert_t vehicle_ctrl_mode_pub;

	///
	/// 路径规划
	__NED Tgt_NED, Tgt_NED_Last;
	__NED Current_NED, Current_NED_Last;

	int  step_simple;
	bool is_at_home;
	bool is_landed;
	bool is_finished;

	timeval time_tick;
	int32_t t_start, t_now, t_task;
	int32_t t_start_hovering;

	bool is_Reached_TgtLocation();
	bool is_Reached_Home();
	bool is_Reached_TgtAlt(float tgt);
	bool is_Reached_TgtAlt(float tgt, float error);

};


#endif	/* __Self_Navigator_H */
