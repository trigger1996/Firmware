/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

#ifndef __RPI_Socket_H
#define __RPI_Socket_H

#include <rpi_config.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/socket_recv.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <poll.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



#define SERVER_IP   "192.168.155.2"         // 有线：192.168.137.1 无线：192.168.155.1，路由器网关192.168.2.100，服务器（接入路由情况下）192.168.2.150
#define SERVER_PORT 40000					// 网关 50000+id号，服务器：40000
#define Tx_Len_Max 144
#define Rx_Len_Max 512

//#define MY_ID	0x01			// ID号，每一架飞机都必须不一样

using namespace std;

class __socket_thread
{
public:
    bool thread_should_exit = false;	 /**< daemon exit flag */
    bool thread_running = false;         /**< daemon status flag */
    int  daemon_task;                    /**< Handle of daemon task / thread */

    void usage(const char *reason);


};

class __socket_rpi
{
public:
    __socket_rpi(const char server_ip[], const uint16_t port);

    int fd;
    socklen_t len;
    struct sockaddr_in ServAddr;	//struct sockaddr和struct sockaddr_in这两个结构体用来处理网络通信的地址。
	struct sockaddr_in MyAddr;

	struct timeval timeout;

    char Tx[Tx_Len_Max];			// 缓冲区
    char Rx[Rx_Len_Max];

	char Send_Frame(char str[]);
    char Receive_Frame(void);
};


class __udp_socket_datalink
{
public:
    __udp_socket_datalink(uint8_t id);

    char refine_Data(char str[]);
	char refine_Data_PlainText(char str[]);

    char Send_Config(__grid current_pt,
					 __AHRS ahrs,
                     float battery,
                     int mission_event);
	char Send_Config(void);				// 如果没有输入量则从ORB中读取数据用来发送
	char Send_Data_Preload(void);
	char Send_PlainText_Cfg_Preload(void);

    char update_Time(void);

    char Recv_to_ORB(__time time,
                     int32_t lat, int32_t lng, int32_t alt,
                     int32_t task);
	char Send_Subscribe(void);


    char Tx[Tx_Len_Max];

    // 接收的数据
    char  Link_Tgt_ID;		// 数据帧中记录的ID
    __time Server_Time;		// 服务器时间
    __grid Nav_Pt[6];		// 航点
    int   Cmd_Extra;		// 额外命令

private:
    // 发送的数据
    unsigned char Flight_ID;

	__grid      current_Pt;
	__AHRS_Link AHRS_cur_Link;
	__AHRS      current_AHRS;
	int         Battery_V_Link;
	float       Battery_V;		// 电池电压
	int         Mission_Event;

	/// uORB相关
	// 地面命令解析发布
	orb_advert_t socket_pub;			// 发布的设备号
	struct socket_recv_s skt_orb;		// 用来发布的结构体

	//电池信息订阅
	int bat_sub_fd;
	struct battery_status_s bat;

	//位置信息订阅
	int lcl_pos_sub_fd;					// 本地位置
	struct vehicle_local_position_s lcl_pos;

	int gbl_pos_sub_fd;					// 全球位置
	struct vehicle_global_position_s gbl_pos;

	// 姿态信息订阅
	int sensor_sub_fd;					// 飞机姿态
	struct vehicle_attitude_s sensor;

};

int  find_Break(char str[], int len);

int socket_thread_main(int argc, char *argv[]);

#endif // __RPI_Socket_H
