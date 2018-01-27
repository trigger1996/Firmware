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
	// 处理对象
	__lidar_driver lidar;
	__optflow      optflow;
	__slam		   slam;

	// 飞机姿态订阅
	int vehicle_att_sub_fd, sensor_combined_sub_fd;
	struct vehicle_attitude_s vehicle_att;
	struct sensor_combined_s  sensor_raw;
	static __AHRS  current_AHRS = { 0.0f };
	static __Vec3f current_Acc  = { 0.0f };

	// 订阅发布变量
	px4_pollfd_struct_t fds;
	int poll_ret;


	/// 初始化激光雷达
	lidar.init();

	// 认为线程从这里开始
	warnx("[rplidar] starting\n");
	rplidar_running = true;

	// 准备订阅
	vehicle_att_sub_fd     = orb_subscribe(ORB_ID(vehicle_attitude));
	sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	while (!rplidar_should_exit) {

		/// 有姿态角的时候再进行光流计算，其他时候睡觉
		fds.fd = vehicle_att_sub_fd;
		fds.events = POLLIN;
		poll_ret = px4_poll(&fds, 1, 50);
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

				current_AHRS.Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;	// pitch
				current_AHRS.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	// roll
				current_AHRS.Yaw   = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;	//yaw

				// 测试代码
				/// 机体坐标系和北东地坐标系类似，正前方X轴，正右方Y轴，正下方Z轴
				/// 旋转正方向由右手定则确定，拇指朝向坐标轴正方向，四指方向为旋转正方向
				//PX4_INFO("Roll:  %f", current_AHRS.Roll);
				//PX4_INFO("Pitch: %f", current_AHRS.Pitch);
				//PX4_INFO("Yaw:   %f", current_AHRS.Yaw);

				/// 进行光流运算
				/// 需要注意的是，图像的X是东西走向，Y是南北走向
				lidar.draw_Frames(current_AHRS.Yaw);
				optflow.run(lidar.raw, lidar.raw_last, false);

				/// 有加速度的时候才进行卡尔曼滤波计算，否则跳过这一步
				fds.fd = sensor_combined_sub_fd;
				fds.events = POLLIN;
				poll_ret = px4_poll(&fds, 1, 50);
				if (poll_ret > 0)
				{
					if (fds.revents & POLLIN)
					{

						orb_copy(ORB_ID(sensor_combined),
								 sensor_combined_sub_fd,
								 &sensor_raw);

						current_Acc.X = sensor_raw.accelerometer_m_s2[0];
						current_Acc.Y = sensor_raw.accelerometer_m_s2[1];
						current_Acc.Z = sensor_raw.accelerometer_m_s2[2];


						//PX4_INFO("Ax: %f, Ay: %f", current_Acc.X,
						//						   current_Acc.Y);

						/// 卡尔曼滤波器，以及作图
						slam.update_Data(optflow.vx, optflow.vy,
										 current_Acc.X, current_Acc.Y,
										 current_AHRS);
						slam.update_Obstacle(lidar.Data_NArray);

						slam.run();

						PX4_INFO("Before: Vx: %f, Vy: %f", optflow.vx, optflow.vy);
						PX4_INFO("Kalman: Vx_Dst: %f, Vy_Dst: %f", slam.vx, slam.vy);
					}
				}

			}
		}


		usleep(50000);	// 100000us
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
}// void rp_task_usage(const char *reason)

