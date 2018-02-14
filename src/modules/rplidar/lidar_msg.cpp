#include <lidar_msg.h>


__lidar_msg::__lidar_msg()
{

}// __lidar_msg::__lidar_msg()

int __lidar_msg::init()
{
	// 准备订阅
	vehicle_att_sub_fd     = orb_subscribe(ORB_ID(vehicle_attitude));
	sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	Acc  = { 0.0f };
	AHRS = { 0.0f };

	return 0;

}// int __lidar_msg::init()

int __lidar_msg::receive_Data(int nms)
{
	bool is_ahrs_ok = false;
	bool is_att_ok = false;

	is_ahrs_ok = update_AHRS(nms / 2);
	is_att_ok  = update_VehicleAtt(nms / 2);

	if (is_ahrs_ok && is_att_ok)
		return 0;
	else
		return -1;

}// int __lidar_msg::receive_Data()

int __lidar_msg::publish_Data()
{

	return 0;

}// int __lidar_msg::publish_Data()

bool __lidar_msg::update_AHRS(int nms)
{
	// 订阅发布变量
	px4_pollfd_struct_t fds;
	int poll_ret;

	fds.fd = vehicle_att_sub_fd;
	fds.events = POLLIN;
	poll_ret = px4_poll(&fds, 1, nms);
	if (poll_ret > 0)
	{
		if (fds.revents & POLLIN)
		{
			orb_copy(ORB_ID(vehicle_attitude),
					 vehicle_att_sub_fd,
					 &vehicle_att);

			// 四元数换算欧垃角
			float q0, q1, q2, q3;
			q0 = vehicle_att.q[0];
			q1 = vehicle_att.q[1];
			q2 = vehicle_att.q[2];
			q3 = vehicle_att.q[3];

			AHRS.Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;									// pitch
			AHRS.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	// roll
			AHRS.Yaw   = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;			//yaw

			return true;
		}
	}

	return false;

}// bool __lidar_msg::update_AHRS(int nms)

bool __lidar_msg::update_VehicleAtt(int nms)
{
	px4_pollfd_struct_t fds;
	int poll_ret;

	fds.fd = sensor_combined_sub_fd;
	fds.events = POLLIN;
	poll_ret = px4_poll(&fds, 1, nms);
	if (poll_ret > 0)
	{
		if (fds.revents & POLLIN)
		{
			orb_copy(ORB_ID(sensor_combined),
					 sensor_combined_sub_fd,
					 &sensor_raw);

			Acc.X = sensor_raw.accelerometer_m_s2[0];
			Acc.Y = sensor_raw.accelerometer_m_s2[1];
			Acc.Z = sensor_raw.accelerometer_m_s2[2];

			return true;
		}
	}

	return false;

}// bool __lidar_msg::update_VehicleAtt(int nms)
