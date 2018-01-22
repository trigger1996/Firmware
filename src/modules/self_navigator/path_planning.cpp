#include "self_navigator.h"

int __self_nav::Flight_Simple()
{
	/*
		简单的飞行任务
		1 解锁
		2 起飞
		3 飞到目标点
		4 悬停
		5 降落
		6 上锁
	*/
	__coordinate_cov cov;
	__NED tgt_ned;

	set_ARM_DISARM(true);
	Takeoff(5.0f);

	/// 设置进入offboard模式
	/// 需要注意的是，commander起飞以后会切掉offboard模式
	while (set_Offboard_Mode(true) != SUCCESS)
	{
		usleep(10000);
	}
	update_HeartBeat();

	update_User_Status(500);
	cov.update_Current_Pos(gbl_pos.lat, gbl_pos.lon, gbl_pos.alt);
	tgt_ned = cov.WGS84toNED(skt_recv.lat, skt_recv.lng, skt_recv.alt); // home position
	get_Tgt_NED(tgt_ned);


	float vx, vy;
	float v_all = 1.5f;
	float theta;

	theta = atan2(tgt_ned.X ,tgt_ned.Y);
	vx = v_all * sin(theta);
	vy = v_all * cos(theta);

	set_Position(0, 0, 10, vx, vy);		// NAN NAN alt vx vy

	int i = 0;
	while (is_Reached_TgtLocation() != true)
	{
		update_User_Status(100);
		update_HeartBeat();
		usleep(300000);
		i++;
	}
	Land();
	return SUCCESS;

}// int __self_nav::flight_Simple()


int __self_nav::get_Tgt_NED(__NED tgt)
{

	Tgt_NED_Last = Tgt_NED;
	Tgt_NED = tgt;

	return SUCCESS;
}// int __self_nav::get_Tgt_NED(__NED tgt)


int __self_nav::get_Current_NED(__NED current)
{

	Current_NED_Last = Current_NED;
	Current_NED = current;

	return SUCCESS;
}// int __self_nav::get_Tgt_NED(__NED tgt)

bool __self_nav::is_Reached_TgtLocation()
{
	// 这里不判断高度
	int d_lat, d_lng;

	if (skt_recv.lat == 0 && skt_recv.lng == 0)
		return true;					// 没发命令就乖乖呆着吧

	d_lat = skt_recv.lat - (int)(gbl_pos.lat * 10000000.0f);
	d_lng = skt_recv.lng - (int)(gbl_pos.lon * 10000000.0f);

	d_lat = abs(d_lat);
	d_lng = abs(d_lng);

	if (d_lat <= GPS_ERROR_THRESHOLD &&
		d_lng <= GPS_ERROR_THRESHOLD)
		return true;
	else
		return false;

}// bool __self_nav::is_NewLocation()

bool __self_nav::is_Reached_Home()
{
	// 这里不判断高度
	int d_lat, d_lng;

	d_lat = skt_recv.lat - (int)(home_pos.lat * 10000000.0f);
	d_lng = skt_recv.lng - (int)(home_pos.lon * 10000000.0f);

	d_lat = abs(d_lat);
	d_lng = abs(d_lng);

	if (d_lat <= GPS_ERROR_THRESHOLD &&
		d_lng <= GPS_ERROR_THRESHOLD)
		return true;
	else
		return false;

}// bool __self_nav::is_NewLocation()

bool __self_nav::is_Reached_TgtAlt(float tgt)
{
	float d_alt;
	d_alt = tgt + home_pos.alt - gbl_pos.alt;
	d_alt = abs(d_alt);

	if (d_alt <  ALT_ERROR_THRESHOLD &&
		d_alt > -ALT_ERROR_THRESHOLD)
		return true;
	else
		return false;

}// bool __self_nav::is_Reached_TgtAlt(alt tgt)

bool __self_nav::is_Reached_TgtAlt(float tgt, float error)
{
	// 高度：单位m
	// 误差：单位m
	if (error < 0.0f)
		error = -error;

	float d_alt;
	d_alt = tgt + home_pos.alt - gbl_pos.alt;
	d_alt = abs(d_alt);

	if (d_alt <  error &&
		d_alt > -error)
		return true;
	else
		return false;

}// bool __self_nav::is_Reached_TgtAlt(alt tgt)
