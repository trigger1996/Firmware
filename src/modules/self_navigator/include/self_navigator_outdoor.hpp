#ifndef __Self_Nav_Outdoor_HPP

#define __Self_Nav_Outdoor_HPP

#include "self_navigator_abs.hpp"

class __self_nav_outdoor: public __self_nav
{
public:

	int set_Position(float x, float y, float z)					// 注意：这里是NED坐标，但是z取了相反数，所以以正值表示天上
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

	}// int set_Position(int x, int y, int z)

	int set_Position(float x, float y, float z, float vx, float vy)
	{
		/// 飞向指定目标
		_pos_sp_triplet.current.valid = true;
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

	}// int set_Position(float x, float y, float z, float vx, float vy)

	int Takeoff(float alt)
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

	}// int Takeoff(float alt)

	int Land()
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

	}// int Land()

};




#endif	/* __Self_Nav_Outdoor_HPP */
