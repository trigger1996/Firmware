#ifndef __RpLidar_H

#define __Rplidar_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/time.h>

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

#include "lidar_driver.h"
#include "slam.h"
#include "optflow.h"

//#pragma GCC diagnostic ignored "-Wunused-function"

// daemon management function.
extern "C" __EXPORT int rplidar_main(int argc, char *argv[]);

// Mainloop of daemon.
int rplidar_thread_main(int argc, char *argv[]);

// Print the correct usage.
void rp_task_usage(const char *reason);


#endif	/* __Rplidar_H */

