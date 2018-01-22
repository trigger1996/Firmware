/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rplidar_task_main.cpp
 * daemon application example for PX4 autopilot
 *
 * @author Yiwei "Wraith" Zheng <529388771@qq.com>
 */
#include "lidar_common.h"


static bool rplidar_should_exit = false;		/**< daemon exit flag */
static bool rplidar_running = false;			/**< daemon status flag */
static int  rplidar_task;						/**< Handle of daemon task / thread */


int rplidar_thread_main(int argc, char *argv[])
{

	__lidar_driver lidar;
	__optflow      optflow;
	int vehicle_att_sub_fd;					// 飞机姿态
	struct vehicle_attitude_s sensor;
	static __AHRS current_AHRS = { 0.0f };

	px4_pollfd_struct_t fds;
	int poll_ret;

	lidar.init();

	warnx("[rplidar] starting\n");
	rplidar_running = true;

	vehicle_att_sub_fd  = orb_subscribe(ORB_ID(vehicle_attitude));

	while (!rplidar_should_exit) {


		/// 订阅数据
		// 姿态角
		fds.fd = vehicle_att_sub_fd;
		fds.events = POLLIN;
		poll_ret = px4_poll(&fds, 1, 200);
		if (poll_ret > 0)
		{
			if (fds.revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_attitude),
						 vehicle_att_sub_fd,
						 &sensor);

				// 四元数换算欧垃角
				float q0, q1, q2, q3;
				q0 = sensor.q[0];
				q1 = sensor.q[1];
				q2 = sensor.q[2];
				q3 = sensor.q[3];

				current_AHRS.Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;	// pitch
				current_AHRS.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	// roll
				current_AHRS.Yaw   = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;	//yaw

				// 测试代码
				//warnx("Yaw: %f", current_AHRS.Yaw);

				lidar.draw_Frames(current_AHRS.Yaw);
				optflow.run(lidar.raw, lidar.raw_last, false);

			}
		}
		else
		{
			warnx("Hello daemon!\n");
		}

		usleep(100000);
	}

	warnx("[daemon] exiting.\n");

	rplidar_running = false;

	return 0;

}// int rplidar_thread_main(int argc, char *argv[])

int rplidar_main(int argc, char *argv[])
{
	if (argc < 2) {
		rp_task_usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rplidar_running) {
			warnx("[rplidar] already running\n");
			/* this is not an error */
			return 0;
		}

		rplidar_should_exit = false;
		rplidar_task = px4_task_spawn_cmd("rplidar",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 108000,
						 rplidar_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		rplidar_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (rplidar_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	rp_task_usage("[rplidar] unrecognized command");
	return 1;

}// int rplidar_main(int argc, char *argv[])


void rp_task_usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: rplidar {start|stop|status} [-p <additional params>]\n\n");
}

