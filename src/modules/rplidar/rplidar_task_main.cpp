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
#include <rplidar_task_main.h>


static bool rplidar_should_exit = false;		/**< daemon exit flag */
static bool rplidar_running = false;			/**< daemon status flag */
static int  rplidar_task;						/**< Handle of daemon task / thread */


int rplidar_thread_main(int argc, char *argv[])
{
	int i = 1;

	// 消息订阅类对象
	__lidar_msg m;

	// 初始化uORB
	m.init();
	// 预装数据
	do {

		int msgstat = FAILED;
		msgstat = m.receive_Data(50);

	} while (m.receive_Data(50) != SUCCESS);

	// 处理对象
	__lidar_driver	lidar;
	__icp			icp;
	__slam			slam(m.AHRS.Yaw);

	/// 初始化激光雷达
	lidar.init();

	///
	/// 预先运行一次
	lidar.run();
	icp.get_Pts(lidar.Data);
	slam.update_Data(lidar.Data, 0, 0, 0);
	slam.update_QuadData(m.Acc.X, m.Acc.Y, m.AHRS);
	slam.yaw = m.AHRS.Yaw * PI / 180.0f;	// 给个初始值，对图像好

	///
	/// 认为线程从这里开始
	warnx("[rplidar] starting\n");
	rplidar_running = true;
	while (!rplidar_should_exit) {

		// 当且仅当传感器有数据的时候才计算，其他时候睡觉
		if (m.receive_Data(50) == SUCCESS) {

			/// 使用ICP以后，不再锁定角度，所以这里可以等数据齐了以后计算
			lidar.run();
			icp.run(lidar.Data, false);
			slam.run(lidar.Data, icp.dx, icp.dy, icp.d_yaw,
					 m.Acc.X, m.Acc.Y,
					 m.AHRS);
			m.publish_SlamData(slam.vx * slam.dt, slam.vy * slam.dt,
							   slam.x, slam.y);

			if (i % 25 == 0)
				imwrite("slam.jpg", slam.Map);				// 50ms

			i++;
			waitKey(30);

		}

		usleep(25000);	// 100000us
	}

	/// 退出线程
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
}// void rp_task_usage(const char *reason)

