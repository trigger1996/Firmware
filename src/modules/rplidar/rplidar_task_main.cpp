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
	// 实际数据
	double vx_dst, vy_dst;	// 速度
	double x, y;			// 距离
	// 飞机姿态订阅
	int vehicle_att_sub_fd, sensor_combined_sub_fd;
	struct vehicle_attitude_s vehicle_att;
	struct sensor_combined_s  sensor_raw;
	static __AHRS  current_AHRS = { 0.0f };
	static __Vec3f current_Acc  = { 0.0f };
	// 订阅发布变量
	px4_pollfd_struct_t fds;
	int poll_ret;
	// 时间变量
	timeval t_now, t_last;
	int dt;

	lidar.init();

	// 认为线程从这里开始
	warnx("[rplidar] starting\n");
	rplidar_running = true;

	// 准备订阅
	vehicle_att_sub_fd     = orb_subscribe(ORB_ID(vehicle_attitude));
	sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	// 初始化时间
	t_now = t_last = { 0 };
	dt = 0;

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

						// 获得时间，用于卡尔曼滤波器
						t_last = t_now;
						gettimeofday(&t_now, NULL);
						dt = t_now.tv_usec - t_last.tv_usec;
						if (dt < 0)
							dt = t_now.tv_usec + 1000000 - t_last.tv_usec;
						//PX4_INFO("t_now: %d", dt);


						/// 卡尔曼滤波器
						kalman_filter(optflow.vx,    optflow.vy,
									  vx_dst,        vy_dst,
									  current_Acc.X, current_Acc.Y,
									  current_AHRS,  (double)dt / 1000);

						//PX4_INFO("Before: Vx: %f, Vy: %f", optflow.vx, optflow.vy);
						//PX4_INFO("Kalman: Vx_Dst: %f, Vy_Dst: %f", vx_dst, vy_dst);

						/// 测出来的速度积分得到距离
						x = x + vx_dst * dt;
						y = y + vy_dst * dt;

					}
				}

			}
		}


		usleep(100000);
	}

	warnx("[daemon] exiting.\n");

	rplidar_running = false;

	return 0;

}// int rplidar_thread_main(int argc, char *argv[])

void kalman_filter(double vx_in,   double vy_in,
				   double &vx_dst, double &vy_dst,
				   double acc_x,   double acc_y,
				   __AHRS ahrs,    double dt)
{
	// 核心代码
	// 输入单位:	速度v:	mm/s
	//			加速度a:	mm/s^2
	//			角度:	deg
	//			时间:	ms

	static double Xx = 0, Xy = 0;
	static double Px = 1, Py = 1;
	static double Kx = 1, Ky = 1;

	double a, b, c;
	double temp;

	const double F = 1;
	const double H = 1;
	const double I = 1;
	const double Q = 0.05  * 0.05;
	const double R = 0.005 * 0.005;

	/// 清零输出
	vx_dst = vy_dst = 0;

	/// 处理时间
	dt = dt / 1000;

	/// 角度转弧度
	a = ahrs.Roll  * PI / 180.0f;		// Roll
	b = ahrs.Pitch * PI / 180.0f;		// Pitch
	c = ahrs.Yaw   * PI / 180.0f;		// Yaw

	/// 先归一化速度
	/// 为什么前面要锁定yaw，这里再换算，这是因为光流会受到旋转的影响，如果前面不锁定虽然可以避免下面的步骤，但是噪声会非常大
	/// 对调x, y速度，因为图像的x, y和实际相反
	//  改代码的时候记得放到外面
	temp = vx_in; vx_in = vy_in; vy_in = temp;
	/// 对地速度->机体速度
	vx_in = vx_in * (sin(a) * sin(c) - cos(a) * sin(b) * cos(c));
	vy_in = vy_in * (sin(a) * cos(c) + cos(a) * sin(b) * sin(c));
	/// 速度修正
	vx_in = vx_in * cos(b);				// 注意这边乘以的是轴向的速度
	vy_in = vy_in * cos(a);

	Xx = F * Xx + acc_x * 1000 * dt;
	Px = F * Px + Q;
	Kx = H * Px / (H * Px + R);
	Xx = Xx + Kx * (vx_in - Xx * H);
	Px = (I - Kx) * Px;

	Xy = F * Xy + acc_y * 1000 * dt;
	Py = F * Py + Q;
	Ky = H * Py /  (H * Py + R);
	Xy = Xy + Ky * (vy_in - Xy * H);
	Py = (I - Ky) * Py;

	vx_dst = Xx;
	vy_dst = Xy;

	// 测试代码，输出卡尔曼增益
	//PX4_INFO("Kalman Gain: Kx: %f, Ky: %f", Kx, Ky);

	// 首先看姿态角，如果倾斜的很厉害就不做计算了
	//if (ahrs.Pitch >= 5 * PI / 180.0f)
	//{
	//	vx_dst = vy_dst = 0;
	//}


}// void kalman_filter(..)



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

