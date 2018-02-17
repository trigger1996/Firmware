#include "self_navigator.h"
#include "self_navigator_abs.hpp"
#include "self_navigator_outdoor.hpp"
#include "self_navigator_indoor.hpp"

/**
 * @file self_navigator_main.cpp
 * Functioning Px4 Offboard Mode
 *
 *
 * !! This is from -- !!
 * http://www.amovauto.com/portal.php?mod=view&aid=7
 *
 */

static orb_advert_t mavlink_log_pub = 0;
static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;

__self_nav_indoor Nav;
int test_flight();			// 试飞

int self_navigator_main(int argc, char *argv[])
{
	if (argc < 2) {
		mavlink_log_critical(&mavlink_log_pub, "[self_navigator]mission command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			mavlink_log_critical(&mavlink_log_pub, "[self_navigator]already running");
			exit(0);
		}

		thread_should_exit = false;
		thread_running = true;
		daemon_task = px4_task_spawn_cmd("self_navigator",
										 SCHED_DEFAULT,
										 SCHED_PRIORITY_MAX - 5,
										 3600,
										 self_navigator_thread_main,
										 &argv[2]);

		return 0;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	mavlink_log_critical(&mavlink_log_pub, "unrecognized command");
	exit(1);
	return 0;
}// int self_navigator_main(int argc, char *argv[])


int self_navigator_thread_main(int argc, char *argv[])
{
	//uint64_t timetick = hrt_absolute_time();
	//int sended = 0;
	__NED tgt_ned = { 0 }, current_ned = { 0 };
	int rc_ch8 = 0;
	__coordinate_cov Cov, Cov_Current;

	Nav.Init();


	//  这里开始获取vehicle_status,有vehicle_status_s里面的system_id和component_id才能发送vehicle_command命令飞机
	while (Nav.update_VehicleStatus() != SUCCESS) { }
	Nav._command.target_system = Nav.vehicle_stat.system_id;//system_id写入
	Nav._command.target_component = Nav.vehicle_stat.component_id;//component_id写入
	int j = 0;
	while (true) {
		if (j <= 500)
			Nav.set_RC(1000, 1000, 1000, 1000);
		else
			Nav.set_RC(1500, 1500, 2000, 1500);
		if (j == 500)
			Nav.set_ARM_DISARM(true);
		j++;
		usleep(20e3);
	}


	//int home_pos_sub = orb_subscribe(ORB_ID(home_position));
	//px4_pollfd_struct_t fds;
	//fds.events = POLLIN;

	/// 更新起飞位置
	while (Nav.update_HomePos() != SUCCESS) { }
	Cov.update_HomePos(Nav.home_pos.lat, Nav.home_pos.lon, Nav.home_pos.alt);

	/// 更新载具信息
	//  这里开始获取vehicle_status,有vehicle_status_s里面的system_id和component_id才能发送vehicle_command命令飞机
	while (Nav.update_VehicleStatus() != SUCCESS) { }


	/// 初始变量设置
	Nav._command.target_system = Nav.vehicle_stat.system_id;//system_id写入
	Nav._command.target_component = Nav.vehicle_stat.component_id;//component_id写入

	/// 设置进入posctl模式
	//  这样在退出offboard的时候会进入posctl，飞机不容易失控
	while (Nav.select_FlightMode(PX4_CUSTOM_MAIN_MODE_POSCTL) != SUCCESS)
	{
		//如果没有跳出表示切换失败,循环回去再切换
		usleep(10000);
	}


	///
	//////////////////////////////////////////////////////////
	Nav.set_ARM_DISARM(true);
	Nav.Takeoff(5.0f);

	/// 设置进入offboard模式
	/// 需要注意的是，commander起飞以后会切掉offboard模式
	while (Nav.set_Offboard_Mode(true) != SUCCESS)
	{
		usleep(10000);
	}
	Nav.update_HeartBeat();


	Nav.set_ARM_DISARM(true);
	Nav.Takeoff(5.0f);
	while (Nav.set_Offboard_Mode(true) != SUCCESS)	{
		usleep(10000);
	}
	Nav.set_Position(0, 0, 10, 0, 0);		// NAN NAN alt vx vy
	int i = 0;
	while (i < 5) {
		Nav.update_HeartBeat();
		i++;
		usleep(250e3);
	}


	/// 拍视频的时候先用这个测试，省事
	//test_flight();

	///
	/// 主循环
	while (true)
	{

		/// 由于有心跳有发送时间限制，所以这里当心订阅别的东西的时候，阻塞导致心跳超时
		/// 测试的时候记得去掉底下的保护
		//if (Nav.update_User_RC(50) == SUCCESS)
		//	rc_ch8 = Nav.rc_in.values[7];		// 遥控器通道8
		//if (rc_ch8 <= SELF_NAV_RC_THRESHOLD)	// 如果遥控器通道8没有大于1750则切掉offboard，在软件方针调试的时候记得切掉这两行
		//	continue;

		/// 测试代码
		//Nav.step_simple = 4;



		///
		/// 以下是offboard的操作
		/// 应急改出手段：将遥控器ch8拨动到1750以下，并且拨动offboard模式控制开关，飞机即可改出，切换为手动模式迫降
		//Nav.Flight_Simple();


		/// 心跳
		//  前面说过了 为了维持offboard模式必须要一个心跳信息最低美秒发布2次offboard_control_mode
		//  我们这里只睡眠100ms 所以理论上每秒发布了10次offboard_control_mode，offboard将不会自动关闭
		Nav.update_HeartBeat();
		usleep(300000);
	}

	mavlink_log_critical(&mavlink_log_pub, "[self_navigator]exiting");

	thread_should_exit = false;
	thread_running = false;

	fflush(stdout);
	return 0;

}// int self_navigator_thread_main(int argc, char *argv[])

int test_flight()
{
	Nav.set_ARM_DISARM(true);
	Nav.Takeoff(5.0f);

	/// 设置进入offboard模式
	/// 需要注意的是，commander起飞以后会切掉offboard模式
	while (Nav.set_Offboard_Mode(true) != SUCCESS)
	{
		usleep(10000);
	}

	Nav.set_Position(0, 0, 10, -1.5, -1.5);		// NAN NAN alt vx vy



	int i = 0;
	while (i < 30)
	{
		Nav.update_HeartBeat();
		usleep(300000);
		i++;
	}

/*
	Nav.set_Position(0, 0, 10, 1.5, 0);		// NAN NAN alt vx vy
	i = 0;
	while (i < 30)
	{
		Nav.update_HeartBeat();
		usleep(300000);
		i++;
	}

	Nav.set_Position(0, 0, 10, 0, 1.5);		// NAN NAN alt vx vy
	i = 0;
	while (i < 30)
	{
		Nav.update_HeartBeat();
		usleep(300000);
		i++;
	}

	Nav.set_Position(0, 0, 10, -1.5, 0);		// NAN NAN alt vx vy
	i = 0;
	while (i < 30)
	{
		Nav.update_HeartBeat();
		usleep(300000);
		i++;
	}

	Nav.set_Position(0, 0, 10, 0, -1.5);		// NAN NAN alt vx vy
	i = 0;
	while (i < 30)
	{
		Nav.update_HeartBeat();
		usleep(300000);
		i++;
	}

	Nav.set_Position(0, 0, 10, 1.5, 0);		// NAN NAN alt vx vy
	i = 0;
	while (i < 30)
	{
		Nav.update_HeartBeat();
		usleep(300000);
		i++;
	}
*/

	Nav.Land();

	return SUCCESS;

}// int test_flight()
