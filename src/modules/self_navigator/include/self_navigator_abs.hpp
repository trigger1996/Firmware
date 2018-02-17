#ifndef __Self_Nav_Abstract_HPP

#define __Self_Nav_Abstract_HPP

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

#include "coordinate_conversion.h"

enum PX4_CUSTOM_MAIN_MODE {
	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	PX4_CUSTOM_MAIN_MODE_ALTCTL,
	PX4_CUSTOM_MAIN_MODE_POSCTL,
	PX4_CUSTOM_MAIN_MODE_AUTO,
	PX4_CUSTOM_MAIN_MODE_ACRO,
	PX4_CUSTOM_MAIN_MODE_OFFBOARD,
	PX4_CUSTOM_MAIN_MODE_STABILIZED,
	PX4_CUSTOM_MAIN_MODE_RATTITUDE,
	PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
	PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET
};

class __self_nav
{
public:


	// 收到数据
	struct vehicle_command_ack_s _ack;

	/// 基本模式选择
	bool is_gps_online;
	bool is_lidar_online;
	//bool is_datalink_online;

	__self_nav()
	{
		// 构造函数

		// Init();

		Tgt_NED.X = Tgt_NED.Y = Tgt_NED.Z = 0.0f;
		Tgt_NED_Last.X = Tgt_NED_Last.Y = Tgt_NED_Last.Z = 0.0f;
		Current_NED.X = Current_NED.Y = Current_NED.Z = 0.0f;
		Current_NED_Last = Current_NED;

		is_landed  = true;
		is_at_home = true;
		step_simple= 0;

	}// __self_nav()

	int Init()
	{
		// 初始化类

		/// 订阅信息号
		home_pos_sub_fd = orb_subscribe(ORB_ID(home_position));
		vehicle_stat_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
		//vehicle_command_ack是发送命令vehicle_command的返回信息,可以查看命令是否执行成功
		vehicle_command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));//订阅vehicle_command_ack

		rc_in_sub_fd = orb_subscribe(ORB_ID(input_rc));

		skt_recv_sub_fd = orb_subscribe(ORB_ID(socket_recv));
		gbl_pos_sub_fd  = orb_subscribe(ORB_ID(vehicle_global_position));
		lcl_pos_sub_fd  = orb_subscribe(ORB_ID(vehicle_local_position));
		bat_sub_fd      = orb_subscribe(ORB_ID(battery_status));

		/// 发布变量设置
		// vehicle_command
		vehicle_command_pub = nullptr;
		_command = { 0 };


		//offboard_control_mode topic的初始化 这个主题必须以每秒2次的频率publish才能让飞机一直维持offboard模式
		offboard_pub = nullptr;
		_ocm = { 0 };
		_ocm.ignore_acceleration_force = true;		//这个很重要,必须要强制关闭加速度控制才能进行速度控制,位置控制

		//从这里开始就是offboard模式的位置控制了,要飞到哪个点，只需要填入相应的xyz，单位是米，坐标系是ned
		pos_sp_triplet_pub = nullptr;
		_pos_sp_triplet = { 0 };
		_pos_sp_triplet.timestamp = hrt_absolute_time();//写当前时间，非必须
		_pos_sp_triplet.current.valid = true;//让3组合的current为可用,且pre和next必须为false,因为位置控制模块的offboard模式没有用到这两个
		_pos_sp_triplet.current.position_valid = true;//这里我们只控制位置,不控制速度、yaw角和加速度 所以只需要开启这个标识位
		_pos_sp_triplet.current.x =  0.0f;
		_pos_sp_triplet.current.y =  0.0f;
		_pos_sp_triplet.current.z = -1.5f;//一开始我们飞到home点上方3米高空悬停
		_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		/// 以下操作仅供参考，不使用
		//要移动飞机，这个type必须是position_setpoint_s::SETPOINT_TYPE_POSITION，其他还有这些type
	//    if (is_takeoff_sp) {
	//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		//这个表示起飞，其实可以不用，直接用position就可以起飞

	//    } else if (is_land_sp) {
	//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;//着陆

	//    } else if (is_loiter_sp) {
	//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;//悬停

	//    } else if (is_idle_sp) {
	//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;//直接让马达速度变成怠速，自己做降落的时候可以使用

	//    } else {
	//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;//这个最关键，我们最常用的
	//    }


		return SUCCESS;

	}// int Init()

	int update_HomePos()
	{
		// 更新起飞位置

		//int home_pos_sub = orb_subscribe(ORB_ID(home_position));
		px4_pollfd_struct_t fds;
		fds.fd = home_pos_sub_fd;
		fds.events = POLLIN;

		int pret = px4_poll(&fds, 1, 1000);//等待home_position位置
		if (pret <= 0)
		{
			warnx("recv _home_pos over 1 second,continue!");
			return FAILED;//如果超时继续获取直到获取成功
		}

		orb_copy(ORB_ID(home_position), home_pos_sub_fd, &home_pos);

		warnx("home pos getted!");//这里表示home点已经获取到
		return SUCCESS;


	}// int update_HomePos()

	int update_VehicleStatus()
	{
		// 更新载具状态

		//int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
		px4_pollfd_struct_t fds;
		fds.fd = vehicle_stat_sub_fd;
		fds.events = POLLIN;

		int pret = px4_poll(&fds, 1, 1000);
		if (pret <= 0)
		{
			warnx("recv _status over 1 second,continue!");
			return FAILED;
		}

		orb_copy(ORB_ID(vehicle_status), vehicle_stat_sub_fd, &vehicle_stat);


		warnx("vehicle_status_s getted!");
		return SUCCESS;		// 这里表示获取成功了


	}// int update_VehicleStatus()


	int update_User_RC(int nms)
	{
		///
		/// \brief rc_in_sub_fd
		///
		px4_pollfd_struct_t fds;
		fds.fd = rc_in_sub_fd;
		fds.events = POLLIN;

		int pret = px4_poll(&fds, 1, nms);
		if (pret > 0)
		{
			orb_copy(ORB_ID(input_rc), rc_in_sub_fd, &rc_in);

			return SUCCESS;
		}

		return FAILED;

	}// int update_User_RC()

	int update_User_Status(int nms)
	{
		///
		/// \brief rc_in_sub_fd
		///
		///
		nms = nms / 4;

		px4_pollfd_struct_t fds;
		fds.fd = rc_in_sub_fd;
		int pret;

		fds.fd = gbl_pos_sub_fd;
		fds.events = POLLIN;

		pret = px4_poll(&fds, 1, nms);
		if (pret > 0)
		{
			orb_copy(ORB_ID(vehicle_global_position), gbl_pos_sub_fd, &gbl_pos);

		}

		fds.fd = lcl_pos_sub_fd;
		fds.events = POLLIN;

		pret = px4_poll(&fds, 1, nms);
		if (pret > 0)
		{
			orb_copy(ORB_ID(vehicle_local_position), lcl_pos_sub_fd, &lcl_pos);
		}


		fds.fd = skt_recv_sub_fd;
		fds.events = POLLIN;

		pret = px4_poll(&fds, 1, nms);
		if (pret > 0)
		{
			orb_copy(ORB_ID(socket_recv), skt_recv_sub_fd, &skt_recv);

		}

		fds.fd = bat_sub_fd;
		fds.events = POLLIN;

		pret = px4_poll(&fds, 1, nms);
		if (pret > 0)
		{
			orb_copy(ORB_ID(socket_recv), bat_sub_fd, &bat);
		}

		return SUCCESS;

	}// int update_User_Status()

	int update_HeartBeat()
	{
		/// 向offboard发送心跳，不然会被踢出
		_ocm.timestamp = hrt_absolute_time();		//这里记得赋给最新的时间
		if (offboard_pub != nullptr) {
			orb_publish(ORB_ID(offboard_control_mode), offboard_pub, &_ocm);
		} else {
			offboard_pub = orb_advertise(ORB_ID(offboard_control_mode), &_ocm);
		}

		return SUCCESS;

	}// int update_HeartBeat()

	int select_FlightMode(enum PX4_CUSTOM_MAIN_MODE mode)
	{

		//发送vehicle_command
		_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;//设置模式命令
		_command.param1 = 1.0f;		//主模式为costom
		_command.param2 = mode;		//二级模式为position control
		_command.param3 = 0.0f;		//三级模式没有！
		if (vehicle_command_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
		} else {
			vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
		}

		px4_pollfd_struct_t fds;
		fds.fd = vehicle_command_ack_sub;
		fds.events = POLLIN;

		//这里循环接受vehicle_command的返回值vehicle_command_ack
		int i = 0;			// 尝试指定次数后跳出
		while (i < 10)
		{
			i++;

			int pret = px4_poll(&fds, 1, 1000);
			if (pret > 0)
			{
				// 如果收到数据则拷贝
				orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);
				if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)	//如果返回值是这个表示切换成功
				{
					warnx("mode %d ok!", mode);
					return SUCCESS;
				}
				else
				{
					warnx("mode %d rejected!", mode);
					return FAILED;
				}
			}

			// 如果超时则继续尝试
			warnx("recv _ack over 1 second,continue!");
		}

		warnx("can't go into posctl mode,continue!");
		return FAILED;


	}// int select_FlightMode(enum PX4_CUSTOM_MAIN_MODE mode)

	int set_Offboard_Mode(bool setOffboard)
	{
		// 这个指令就是进入offboard模式的指令

		// 更新心跳
		_ocm.timestamp = hrt_absolute_time();
		if (offboard_pub != nullptr) {
			orb_publish(ORB_ID(offboard_control_mode), offboard_pub, &_ocm);
		} else {
			offboard_pub = orb_advertise(ORB_ID(offboard_control_mode), &_ocm);
		}

		// 发送offboard模式
		_command.command = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;
		//但是在这之前，我们需要先发布一次offboard_control_mode，否则无法进入offboard，常见上面的代码
		if (setOffboard == true)
			_command.param1 = 1.0f;	//0.0f表示关闭offboard 1.0f表示开启
		else
			_command.param1 = 0.0f;
		if (vehicle_command_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
		} else {
			vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
		}

		//这里取返回值，跟上面的代码一样
		px4_pollfd_struct_t fds;
		fds.fd = vehicle_command_ack_sub;
		fds.events = POLLIN;

		int i = 0;			// 尝试指定次数后跳出
		while (i < 10)
		{
			i++;

			int pret = px4_poll(&fds, 1, 1000);
			if (pret > 0)
			{
				// 如果收到数据则拷贝
				orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);
				if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)	//如果返回值是这个表示切换成功
				{
					warnx("enable/disable offboard mode ok!");
					return SUCCESS;
				}
				else
				{
					warnx("enable/disable offboard mode rejected!");
					return FAILED;
				}
			}

			// 如果超时则继续尝试
			warnx("recv _ack over 1 second,continue!");
		}

		warnx("can't enable/disable into offboard mode,continue!");
		return FAILED;


	}// int set_Offboard_Mode()


	int set_ARM_DISARM(bool setARM)
	{
		_command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;//发送指令arm
		if (setARM == true)
			_command.param1 = 1.0f;//1.0为解锁 0.0为加锁
		else
			_command.param1 = 0.0f;

		if (vehicle_command_pub != nullptr) {//发布消息
			orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
		} else {
			vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
		}

		//这里取返回值，跟上面的代码一样
		px4_pollfd_struct_t fds;
		fds.fd = vehicle_command_ack_sub;
		fds.events = POLLIN;

		int i = 0;			// 尝试指定次数后跳出
		while (i < 10)
		{
			i++;

			int pret = px4_poll(&fds, 1, 1000);
			if (pret > 0)
			{
				// 如果收到数据则拷贝
				orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);
				if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)	//如果返回值是这个表示切换成功
				{
					warnx("arm/disarm ok!");
					return SUCCESS;
				}
				else
				{
					warnx("arm/disarm mode rejected!");
					return FAILED;
				}
			}

			// 如果超时则继续尝试
			warnx("recv _ack over 1 second,continue!");
		}

		warnx("can't arm/disarm");
		return FAILED;

	}// int set_ARM_DISARM(bool setARM)

	virtual int set_Position(float x, float y, float z, float vx, float vy);
	virtual int Takeoff(float alt);
	virtual int Land();

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

protected:

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



#endif	/* __Self_Nav_Abstract_HPP */
