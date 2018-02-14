#ifndef LIDAR_MSG_H
#define LIDAR_MSG_H

#include <rpi_config.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/rp_slam.h>
#include <poll.h>

class __lidar_msg
{
public:
	__lidar_msg();

	__AHRS  AHRS;
	__Vec3f Acc;

	int init();

	int receive_Data(int nms);
	int publish_SlamData(double dx, double dy, double x, double y);

private:

	// 飞机姿态订阅
	int vehicle_att_sub_fd;
	int sensor_combined_sub_fd;

	// 激光雷达数据发布
	orb_advert_t slam_pub;

	struct vehicle_attitude_s vehicle_att;
	struct sensor_combined_s  sensor_raw;

	struct rp_slam_s slam_data;

	bool update_AHRS(int nms);
	bool update_VehicleAtt(int nms);

};

#endif // LIDAR_MSG_H
