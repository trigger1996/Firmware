#ifndef __RpLidar_H

#define __RpLidar_H

//#pragma GCC diagnostic ignored "-Wunused-function"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/time.h>

#include <vector>
#include <iostream>

#include <rpi_config.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <poll.h>

#include <rplidar.h>

///
/// SLAM使用的变量
typedef struct {
	_u8   Quality;
	float Angle;
	float Dst;

} __scandot;

// daemon management function.
extern "C" __EXPORT int rplidar_main(int argc, char *argv[]);

// Mainloop of daemon.
int rplidar_thread_main(int argc, char *argv[]);

// Print the correct usage.
void rp_task_usage(const char *reason);


#endif	/* __RpLidar_H */

