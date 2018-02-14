#include "self_navigator.h"


__self_nav::__self_nav()
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

}// __self_nav::__self_nav()

int __self_nav::Init()
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
}// int __self_nav::Init()

int __self_nav::update_HomePos()
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


}// int __self_nav::update_HomePos()

int __self_nav::update_VehicleStatus()
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


}// int __self_nav::update_VehicleStatus()

int __self_nav::update_User_RC(int nms)
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

}// int __self_nav::update_User_RC()

int __self_nav::update_User_Status(int nms)
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

}// int __self_nav::update_User_Status()

int __self_nav::update_HeartBeat()
{
	/// 向offboard发送心跳，不然会被踢出
	_ocm.timestamp = hrt_absolute_time();		//这里记得赋给最新的时间
	if (offboard_pub != nullptr) {
		orb_publish(ORB_ID(offboard_control_mode), offboard_pub, &_ocm);
	} else {
		offboard_pub = orb_advertise(ORB_ID(offboard_control_mode), &_ocm);
	}

	return SUCCESS;

}// int __self_nav::update_HeartBeat()

int __self_nav::select_FlightMode()
{

	//发送vehicle_command
	_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;//设置模式命令
	_command.param1 = 1.0f;//主模式为costom
	_command.param2 = 3.0f;//二级模式为position control
	_command.param3 = 0.0f;//三级模式没有！
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
				warnx("posctrl mode ok!");
				return SUCCESS;
			}
			else
			{
				warnx("posctrl mode rejected!");
				return FAILED;
			}
		}

		// 如果超时则继续尝试
		warnx("recv _ack over 1 second,continue!");
	}

	warnx("can't go into posctl mode,continue!");
	return FAILED;


}// int __self_nav::select_FlightMode()

int __self_nav::set_Offboard_Mode(bool setOffboard)
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


}// int __self_nav::set_Offboard_Mode()

int __self_nav::set_ARM_DISARM(bool setARM)
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

}// int __self_nav::set_ARM_DISARM(bool setARM)

int __self_nav::set_Position(float x, float y, float z)
{
	/// 飞向指定目标
	//飞到这个坐标
	_pos_sp_triplet.current.x =  x;
	_pos_sp_triplet.current.y =  y;
	_pos_sp_triplet.current.z = -z;

	warnx("position");
	if (pos_sp_triplet_pub == nullptr) {
		pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	} else {
		orb_publish(ORB_ID(position_setpoint_triplet), pos_sp_triplet_pub, &_pos_sp_triplet);
	}


	return SUCCESS;

}// int __self_nav::set_Position(int x, int y, int z)

int __self_nav::set_Position(float x, float y, float z, float vx, float vy)
{
	/// 飞向指定目标

	_ctrl_mode.flag_control_position_enabled = false;
	_ctrl_mode.flag_control_velocity_enabled = true;
	_ctrl_mode.flag_control_altitude_enabled = true;
	if (vehicle_ctrl_mode_pub == nullptr) {
		vehicle_ctrl_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &_ctrl_mode);
	} else {
		orb_publish(ORB_ID(position_setpoint_triplet), vehicle_ctrl_mode_pub, &_ctrl_mode);
	}

	_pos_sp_triplet.current.position_valid = false;
	_pos_sp_triplet.current.velocity_valid = true;
	_pos_sp_triplet.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_BODY_NED;

	_pos_sp_triplet.current.alt_valid = true;

	_pos_sp_triplet.current.vx  = vx;
	_pos_sp_triplet.current.vy  = vy;
	_pos_sp_triplet.current.alt = z + home_pos.alt;
	_pos_sp_triplet.current.z = -z;

	warnx("velocity");
	if (pos_sp_triplet_pub == nullptr) {
		pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	} else {
		orb_publish(ORB_ID(position_setpoint_triplet), pos_sp_triplet_pub, &_pos_sp_triplet);
	}

	return SUCCESS;

}// int __self_nav::set_Position(float x, float y, float z, float vx, float vy)
\
int __self_nav::Takeoff(float alt)
{
	// alt: AMSL，单位: m
	int i = 0;

	while (i < 50)
	{
		i++;

		_command.timestamp = hrt_absolute_time();
		_command.param5 = NAN;
		_command.param6 = NAN;
		/* minimum pitch */
		_command.param1 = NAN;
		_command.param2 = NAN;
		_command.param3 = NAN;
		_command.param4 = NAN;
		_command.param7 = alt + home_pos.alt;
		_command.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
		_command.target_system = (uint8_t)vehicle_stat.system_id;;
		_command.target_component = (uint8_t)vehicle_stat.component_id;

		if (vehicle_command_pub != nullptr) {//发布消息
			orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
		} else {
			vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
		}

		//这里取返回值，跟上面的代码一样
		px4_pollfd_struct_t fds;
		fds.fd = vehicle_command_ack_sub;
		fds.events = POLLIN;

		int pret = px4_poll(&fds, 1, 500);
		if (pret > 0)
		{
			// 如果收到数据则拷贝
			orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);
			if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)	//如果返回值是这个表示切换成功
			{
				warnx("takeoff ok!");
				return SUCCESS;
			}
			else
			{
				warnx("takeoff rejected!, retrying..., %d", i);
			}
		}

		update_HeartBeat();
		usleep(300000);		// 300ms
	}

	warnx("takeoff failed!");
	return FAILED;

}// int __self_nav::Takeoff(float alt)

int __self_nav::Land()
{
	int i = 0;

	while (i < 25)
	{
		i++;

		_command.timestamp = 0;
		_command.param5 = NAN;
		_command.param6 = NAN;
		/* minimum pitch */
		_command.param1 = NAN;
		_command.param2 = NAN;
		_command.param3 = NAN;
		_command.param4 = NAN;
		_command.param7 = NAN;
		_command.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
		_command.target_system = (uint8_t)vehicle_stat.system_id;;
		_command.target_component = (uint8_t)vehicle_stat.component_id;

		if (vehicle_command_pub != nullptr) {//发布消息
			orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
		} else {
			vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
		}

		//这里取返回值，跟上面的代码一样
		px4_pollfd_struct_t fds;
		fds.fd = vehicle_command_ack_sub;
		fds.events = POLLIN;

		int pret = px4_poll(&fds, 1, 500);
		if (pret > 0)
		{
			// 如果收到数据则拷贝
			orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);
			if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)	//如果返回值是这个表示切换成功
			{
				warnx("land ok!");
				return SUCCESS;
			}
			else
			{
				warnx("land rejected!, retrying..., %d", i);
			}
		}

		update_HeartBeat();
		usleep(50000);		// 50ms
	}

	///
	/// 新版的不能使用降落命令了，直接改成设点
	set_Position(NAN, NAN, 0 - 1.5f, 0, 0);
	warnx("land command failed, attempt to land by setting nav-pt...");
	//warnx("land failed!");
	return STANDBY;

}// int __self_nav::Land()

