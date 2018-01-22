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


#include "usart.h"

__usart_report USART;

/**
 * daemon management function.
 */
extern "C" __EXPORT int usart_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int usart_thread_main(int argc, char *argv[]);


int usart_thread_main(int argc, char *argv[])
{

	if (argc < 2) {
		errx(1, "[usart_report]need a serial port name as argument");
		USART.usage("eg:");
	}
	const char *uart_name = argv[1];
	warnx("[usart_report]opening port %s", uart_name);
	char data = '0';
	char buffer[4] = "";
/*
	* TELEM1 : /dev/ttyS1
	* TELEM2 : /dev/ttyS2
	* GPS : /dev/ttyS3
	* NSH : /dev/ttyS5
	* SERIAL4: /dev/ttyS6
	* N/A : /dev/ttyS4
	* IO DEBUG (RX only):/dev/ttyS0
*/
	int uart_read = USART.init(uart_name);
	if(false == uart_read)
		return -1;
	if(false == USART.set_Baudrate(9600)){
		printf("[usart_report]set_uart_baudrate is failed\n");
		return -1;
	}
	PX4_INFO("[usart_report]uart init is successful\n");

	warnx("[usart_report] daemon starting\n");

	USART.thread_running = true;

	// 订阅
	// 姿态信息订阅
	int sensor_sub_fd;					// 飞机姿态
	struct vehicle_attitude_s sensor;

	int ctrl_state_fd;
	struct control_state_s ctrl_state;

	/// 角度
	__AHRS current_AHRS;
	float q0, q1, q2, q3;
	/// 角速度
	__Vec3f w = { 0 };
	/// 加速度
	__Vec3f acc = { 0 };
	/// 速度
	__Vec3f v = { 0 };

	px4_pollfd_struct_t fds;
	int poll_ret;

	sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	ctrl_state_fd = orb_subscribe(ORB_ID(control_state));
	while (!USART.thread_should_exit) {

		bool is_vehicle_att_updated = false;
		bool is_ctrl_stat_updated   = false;

		/// 欧拉角和角速度
		fds.fd = sensor_sub_fd;
		fds.events = POLLIN;
		poll_ret = px4_poll(&fds, 1, 25);
		if (poll_ret > 0)
		{
			if (fds.revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_attitude),
						 sensor_sub_fd,
						 &sensor);

				// 四元数换算欧垃角
				q0 = sensor.q[0];
				q1 = sensor.q[1];
				q2 = sensor.q[2];
				q3 = sensor.q[3];

				current_AHRS.Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;									// pitch
				current_AHRS.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	// roll
				current_AHRS.Yaw   = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;			//yaw

				w.X = sensor.rollspeed;
				w.Y = sensor.pitchspeed;
				w.Z = sensor.yawspeed;

				is_vehicle_att_updated = true;
			}
			else
				is_vehicle_att_updated = false;

		}
		else
			is_vehicle_att_updated = false;

		/// 速度和加速度
		fds.fd = ctrl_state_fd;
		fds.events = POLLIN;
		poll_ret = px4_poll(&fds, 1, 25);
		if (poll_ret > 0)
		{
			if (fds.revents & POLLIN)
			{
				orb_copy(ORB_ID(control_state),
						 ctrl_state_fd,
						 &ctrl_state);

				acc.X = ctrl_state.x_acc;
				acc.Y = ctrl_state.y_acc;
				acc.Z = ctrl_state.z_acc;

				v.X = ctrl_state.x_vel;
				v.Y = ctrl_state.y_vel;
				v.Z = ctrl_state.z_vel;

				is_ctrl_stat_updated = true;
			}
			else
				is_ctrl_stat_updated = false;

		}
		else
			is_ctrl_stat_updated = false;



		// 数据协议和飞机用的明语是一样的
		// Pitch:<数据> Roll:<数据> Yaw:<数据>{两个空格}
		if (is_vehicle_att_updated == true &&
			is_ctrl_stat_updated == true)
		{
			char str[256] = { 0 };
			sprintf(str, "Pitch:%d Roll:%d Yaw:%d Wx:%d Wy:%d Wz:%d Ax:%d Ay:%d Az:%d Vx:%d Vy:%d Vz:%d  ",			// 一个%以7个字符算，则有139个字符
					(int)(current_AHRS.Pitch * 1000.0f), (int)(current_AHRS.Roll * 1000.0f), (int)(current_AHRS.Yaw * 1000.0f),
					(int)(w.X * 1000.0f),                (int)(w.Y * 1000.0f),               (int)(w.Z * 1000.0f),
					(int)(acc.X * 1000.0f),              (int)(acc.Y * 1000.0f),             (int)(acc.Z * 1000.0f),
					(int)(v.X * 1000.0f),                (int)(v.Y * 1000.0f),               (int)(v.Z * 1000.0f));

			USART.send_nBytes(str, strlen(str));

			// Debug
			//PX4_INFO("|%s |\n", str);

			usleep(25000);				//100ms, 其实真正只需要139 / 9600秒，即约15ms
			is_vehicle_att_updated = false;
			is_ctrl_stat_updated   = false;
		}

		//warnx("Hello daemon!\n");

	}

	warnx("[usart_report] exiting.\n");

	USART.thread_running = false;

	return 0;
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int usart_main(int argc, char *argv[])
{
	if (argc < 2) {
		USART.usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (USART.thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		USART.thread_should_exit = false;
		USART.daemon_task = px4_task_spawn_cmd("usart_report",
												SCHED_DEFAULT,
												SCHED_PRIORITY_DEFAULT - 5,
												2000,
												usart_thread_main,
												(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		USART.thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (USART.thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	USART.usage("unrecognized command");
	return 1;
}



/**
 * Print the correct usage.
 */
void __usart_report::usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: usart {start|stop|status} [<device name>]\n\n");
	warnx("TELEM1 : /dev/ttyS1");
	warnx("TELEM2 : /dev/ttyS2");
	warnx("GPS    : /dev/ttyS3");
	warnx("N/A : /dev/ttyS4");
	warnx("NSH : /dev/ttyS5");
	warnx("SERIAL4: /dev/ttyS6");
	warnx("IO DEBUG (RX only):/dev/ttyS0");
	warnx("e.g: usart start /dev/ttyS2");


}// void __usart_report::usage(const char *reason)


