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



#endif	/* __Self_Navigator_H */
