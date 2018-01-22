#include "rpi_socket.h"

/*
树莓派-服务器数据链：
    关于校验：这边使用UDP通信，校验问题可以不管，而且民用也不需要加密

    数据帧格式定义：
    所有数据全部集合成一帧发送
    这里包括树莓派-服务器以及服务器-树莓派
    这里两种数据帧

    Server->Rpi
    帧头：									1字节，  0xAF
    帧长：									1字节，  85
    功能编号：								1字节，  0x01
    飞机ID号									1字节，  0x01->0xFF
    时间同步									5字节，  Hour Minute Second mSecond_H mSecond_L
    航点1 航点2 航点3 航点4 航点5 航点6			72字节， 每个航点4Byte Lat、4Byte Lng、4Byte Alt，所有数据 经度*10^7，高度精确到毫米（乘以1000发送）
    额外命令									4字节，  0x00无功能，0x01开始编号

    Rpi->Server
    帧头：									1字节，  0xAF
    帧长：									1字节，  28
    功能编号：								1字节，  0x02
    飞机ID号：								1字节，  0x01->0xFF
    当前位置：								12字节， Lat * 4， Lng * 4， Alt * 4，所有数据 经度*10^7，高度精确到毫米（乘以1000发送）
    姿态角：									6字节，  Pitch * 2， Roll * 2， Yaw * 2，所有数据*100发送
    电池数据：								2字节，
    任务事件：								4字节，

	明语通信：
		由于Java和C++在字符串这边太多东西不兼容，导致上述数据链发过去直接爆炸，0xAF能收成八竿子也打不着的-17
		所以对于手机平台，直接改为明语通信，反正现在加密和安全的问题不用管，毕竟不是军用：

		通信数据、精度和上面完全一样
		发送方式做出改变
		数据格式如下，记住按格式保留所有空格
		数据全部用字符串表示，比如24.1234就是一个str[6]

		Rpi->Server
			ID:<数据> Lat:<数据> Lng:<数据> Alt:<数据> Pitch:<数据> Roll:<数据> Yaw:<数据> Bat:<数据> Tsk:<数据>{两个空格}

		Server->Rpi:
			ID:<数据> Hour:<数据> Min:<数据> Sec:<数据> mSec:<数据> Lat:<数据> Lng:<数据> Alt:<数据> Tsk:<数据>{两个空格}

		注意：
			1 服务器->飞控现在只有一个航点
			2 两种数据帧最后都加两个空格，用来表示结束
*/
__udp_socket_datalink::__udp_socket_datalink(uint8_t id)
{
    Flight_ID = id;


	/// uORB相关
	// 发布准备
	memset(&skt_orb, 0, sizeof(skt_orb));
	socket_pub = orb_advertise(ORB_ID(socket_recv), &skt_orb);

	// 订阅准备
	bat_sub_fd     = orb_subscribe(ORB_ID(battery_status));
	gbl_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	lcl_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	sensor_sub_fd  = orb_subscribe(ORB_ID(vehicle_attitude));

	// 设置更新周期
	orb_set_interval(bat_sub_fd,     200);		// ms
	orb_set_interval(gbl_pos_sub_fd, 200);
	orb_set_interval(lcl_pos_sub_fd, 200);
	orb_set_interval(sensor_sub_fd,  200);

}//__udp_socket_datalink::__udp_socket_datalink


char __udp_socket_datalink::refine_Data(char str[])
{
    // 从数据链中提取数据
    int int_transfer;
    unsigned char temp_1, temp_2, temp_3, temp_4;
    int i, j;

    Link_Tgt_ID = str[3];

	if (Link_Tgt_ID != Flight_ID) return FAILED;

    // 时间同步
    Server_Time.Hour   = str[4];
    Server_Time.Minute = str[5];
    Server_Time.Sec    = str[6];
    int_transfer = str[7] * 256 + str[8];
    Server_Time.mSec   = int_transfer;

    // 航点同步
    for (i = 9, j = 0; i < 81 && j < 6; i += 12, j++)
    {
        // Lat
        temp_1 = str[i];
        temp_2 = str[i + 1];
        temp_3 = str[i + 2];
        temp_4 = str[i + 3];
        int_transfer = (temp_1 << 24) + (temp_2 << 16) + (temp_3 << 8) + temp_4;
        Nav_Pt[j].Lat = int_transfer;


        // Lng
        temp_1 = str[i + 4];
        temp_2 = str[i + 5];
        temp_3 = str[i + 6];
        temp_4 = str[i + 7];
        int_transfer = (temp_1 << 24) + (temp_2 << 16) + (temp_3 << 8) + temp_4;
        Nav_Pt[j].Lng = int_transfer;


        // Alt
        temp_1 = str[i + 8];
        temp_2 = str[i + 9];
        temp_3 = str[i + 10];
        temp_4 = str[i + 11];
        int_transfer = (temp_1 << 24) + (temp_2 << 16) + (temp_3 << 8) + temp_4;
        Nav_Pt[j].Alt = int_transfer;

    }

    temp_1 = str[81];
    temp_2 = str[82];
    temp_3 = str[83];
    temp_4 = str[84];
    int_transfer = (temp_1 << 24) + (temp_2 << 16) + (temp_3 << 8) + temp_4;
    Cmd_Extra = int_transfer;

    return SUCCESS;

}//__rpi_socket_datalink::refine_Data


char __udp_socket_datalink::Send_Config(__grid current_pt,
										__AHRS ahrs,
                                        float battery,
                                        int mission_event)
{
    // 发送数据配置
    current_Pt   = current_pt;

    current_AHRS = ahrs;
    AHRS_cur_Link.Pitch = (int)(current_AHRS.Pitch * 100);
    AHRS_cur_Link.Roll  = (int)(current_AHRS.Roll  * 100);
    AHRS_cur_Link.Yaw   = (int)(current_AHRS.Yaw   * 100);

	Battery_V = battery;
	Battery_V_Link = (int)(Battery_V * 100.0f);

    Mission_Event = mission_event;

	Send_Data_Preload();

    return SUCCESS;

}//__udp_socket_datalink::Send_Config

char __udp_socket_datalink::Send_Config(void)
{
	Send_Subscribe();

	//其实和前面的没差
	//current_Pt   = current_pt;

	AHRS_cur_Link.Pitch = (int)(current_AHRS.Pitch * 100);
	AHRS_cur_Link.Roll  = (int)(current_AHRS.Roll  * 100);
	AHRS_cur_Link.Yaw   = (int)(current_AHRS.Yaw   * 100);

	Battery_V_Link = (int)(Battery_V * 100.0f);

	/// 注意这个参数，到时候还得再弄过来一下
	//Mission_Event = mission_event;

	Send_Data_Preload();

	return SUCCESS;
}

char __udp_socket_datalink::Send_Data_Preload(void)
{
    // 发送数据
	// 需要注意的是，这些数据在发送之前都已经处理好过了
    int int_temp;

    // 帧头
    Tx[0]  = 0xAF;

    // 帧长
    Tx[1]  = 28;

    // 功能
    Tx[2]  = 0x02;

    // 飞机ID
    Tx[3]  = Flight_ID;


    // 当前位置
    // Lat
    int_temp  = current_Pt.Lat;
    Tx[4]     = (int_temp >> 24);
    Tx[5]     = (int_temp << 8)  >> 24;
    Tx[6]     = (int_temp << 16) >> 24;
    Tx[7]     = (int_temp << 24) >> 24;

    // Lng
    int_temp = current_Pt.Lng;
    Tx[8]    =  int_temp >> 24;
    Tx[9]    = (int_temp << 8)  >> 24;
    Tx[10]   = (int_temp << 16) >> 24;
    Tx[11]   = (int_temp << 24) >> 24;

    // Alt
    int_temp = current_Pt.Alt;
    Tx[12]   =  int_temp >> 24;
    Tx[13]   = (int_temp << 8)  >> 24;
    Tx[14]   = (int_temp << 16) >> 24;
    Tx[15]   = (int_temp << 24) >> 24;

    // 姿态角
    // Pitch
    int_temp = AHRS_cur_Link.Pitch;
    Tx[16]   = int_temp / 256;
    Tx[17]   = int_temp % 256;

    // Roll
    int_temp = AHRS_cur_Link.Roll;
    Tx[18]   = int_temp / 256;
    Tx[19]   = int_temp % 256;

    // Yaw
    int_temp = AHRS_cur_Link.Yaw;
    Tx[20]   = int_temp / 256;
    Tx[21]   = int_temp % 256;

    // 电池
	int_temp = Battery_V_Link;
    Tx[22]   = int_temp / 256;
    Tx[23]   = int_temp % 256;

    // 任务事件
    int_temp = Mission_Event;
    Tx[24]   =  int_temp >> 24;
    Tx[25]   = (int_temp << 8)  >> 24;
    Tx[26]   = (int_temp << 16) >> 24;
    Tx[27]   = (int_temp << 24) >> 24;

    return SUCCESS;

}//__udp_socket_datalink::send_Data

char __udp_socket_datalink::Send_PlainText_Cfg_Preload(void)
{
	/*

		Rpi->Server
			ID:<数据> Lat:<数据> Lng:<数据> Alt:<数据> Pitch:<数据> Roll:<数据> Yaw:<数据> Bat:<数据> Tsk:<数据>

	*/

	Send_Subscribe();

	//其实和前面的没差
	//current_Pt   = current_pt;

	AHRS_cur_Link.Pitch = (int)(current_AHRS.Pitch * 100);
	AHRS_cur_Link.Roll  = (int)(current_AHRS.Roll  * 100);
	AHRS_cur_Link.Yaw   = (int)(current_AHRS.Yaw   * 100);

	Battery_V_Link = (int)(Battery_V * 100.0f);

	/// 注意这个参数，到时候还得再弄过来一下
	//Mission_Event = mission_event;

	sprintf(Tx,
			"ID:%d Lat:%d Lng:%d Alt:%d Pitch:%d Roll:%d Yaw:%d Bat:%d Tsk:%d  ",
			Flight_ID,
			current_Pt.Lat, current_Pt.Lng, current_Pt.Alt,
			AHRS_cur_Link.Pitch, AHRS_cur_Link.Roll, AHRS_cur_Link.Yaw,
			Battery_V_Link,
			Mission_Event);

	return SUCCESS;

}//__udp_socket_datalink::Send_PlainText_Cfg_Preload

char __udp_socket_datalink::refine_Data_PlainText(char str[])
{
	char *offset;					// 每段开始的指针位置
	int  end;						// 每段空格的位置
	char num_temp[15] = { 0 };		// 用来存放数字的地方

	int  i;							// 计数变量
	int  id;						// 本帧的ID缓存位置
	const int max_num_len = 15;		// 最大的数字字符串长度，这个是个人工估算的参考值，用在找空格的时候

	//先查ID
	offset = strstr(str, "ID:") + strlen("ID:");
	if (offset == NULL) return FAILED;	// 第一个数据查一下，如果不行就爆炸
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}

	id = atoi(num_temp);
	if (id != Flight_ID) return FAILED;

	// 时间
	// Hour
	offset = strstr(str, "Hour:") + strlen("Hour:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Server_Time.Hour = atoi(num_temp);

	// Minute
	offset = strstr(str, "Min:") + strlen("Min:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Server_Time.Minute = atoi(num_temp);

	// Second
	offset = strstr(str, "Sec:") + strlen("Sec:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Server_Time.Sec = atoi(num_temp);

	// mSecond
	offset = strstr(str, "mSec:") + strlen("mSec:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Server_Time.mSec = atoi(num_temp);

	// Lat
	offset = strstr(str, "Lat:") + strlen("Lat:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Nav_Pt[0].Lat = atoi(num_temp);

	// Lng
	offset = strstr(str, "Lng:") + strlen("Lng:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Nav_Pt[0].Lng = atoi(num_temp);

	// Alt
	offset = strstr(str, "Alt:") + strlen("Alt:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Nav_Pt[0].Alt = atoi(num_temp);

	// Task
	offset = strstr(str, "Tsk:") + strlen("Tsk:");
	end = find_Break(offset, max_num_len);

	memset(num_temp, 0, sizeof(num_temp));
	for (i = 0; i < end; i++)
	{
		num_temp[i] = offset[i];
	}
	Cmd_Extra = atoi(num_temp);

	return SUCCESS;

}// __udp_socket_datalink::refine_Data_PlainText

int  find_Break(char str[], int len)
{
	// 专门用来找空格的函数，如果找不到则返回0，返回距离开始处最近的空格
	// 请确保第一位不是空格，不然这个函数不起作用
	int i = 0;

	for (i = 0; i < len; i++)
	{
		if (str[i] == 32) return i;
	}

	return 0;

}// find_Break


/// 以下代码为了兼容Px4，在uORB内对数据完成订阅和发布
char __udp_socket_datalink::Recv_to_ORB(__time time,
                                        int32_t lat, int32_t lng, int32_t alt,
                                        int32_t task)
{
	// 将地面发过来的数据整理到ORB内
	// 关于skt_orb结构提，详见msg/socket_recv.msg

	//memset(&skt_orb, 0, sizeof(skt_orb));

	skt_orb.hour   = time.Hour;
	skt_orb.minute = time.Minute;
	skt_orb.sec    = time.Sec;
	skt_orb.msec   = time.mSec;

	skt_orb.lng    = lng;
	skt_orb.lat    = lat;
	skt_orb.alt    = alt;

	skt_orb.task   = task;

	// 发布数据
	orb_publish(ORB_ID(socket_recv), socket_pub, &skt_orb);

	return SUCCESS;

}// __udp_socket_datalink::Recv_to_ORB

char __udp_socket_datalink::Send_Subscribe(void)
{
	px4_pollfd_struct_t fds;
	int poll_ret;

	float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;

	// 订阅下载电池数据
	fds.fd = bat_sub_fd;
	fds.events = POLLIN;
	poll_ret = px4_poll(&fds, 1, 200);		// 最后一个参数单位ms
	if (poll_ret > 0)						// 目标有更新
	{
		if (fds.revents & POLLIN)			// 如果有数据变化则读取，POLL的操作详见px4_simple_app
		{
			orb_copy(ORB_ID(battery_status), bat_sub_fd, &bat);
			Battery_V = bat.voltage_v;
			Battery_V_Link = (int)(Battery_V * 100.0f);
		}
	}

	// 下载订阅下载位置信息
	// 全球坐标
	fds.fd = gbl_pos_sub_fd;
	fds.events = POLLIN;
	poll_ret = px4_poll(&fds, 1, 200);
	if (poll_ret > 0)
	{
		if (fds.revents & POLLIN)
		{
			orb_copy(ORB_ID(vehicle_global_position),
					 gbl_pos_sub_fd,
					 &gbl_pos);

			current_Pt.Lng = (int)(gbl_pos.lon * 10000000);
			current_Pt.Lat = (int)(gbl_pos.lat * 10000000);
			current_Pt.Alt = (int)(gbl_pos.alt * 1000);
		}
	}

	// 本地坐标
	fds.fd = lcl_pos_sub_fd;
	fds.events = POLLIN;
	poll_ret = px4_poll(&fds, 1, 200);
	if (poll_ret > 0)
	{
		if (fds.revents & POLLIN)
		{
			orb_copy(ORB_ID(vehicle_local_position),
					 lcl_pos_sub_fd,
					 &lcl_pos);

		}
	}

	// 姿态角
	fds.fd = sensor_sub_fd;
	fds.events = POLLIN;
	poll_ret = px4_poll(&fds, 1, 200);
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

			current_AHRS.Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;	// pitch
			current_AHRS.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	// roll
			current_AHRS.Yaw   = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;	//yaw
		}
	}

	/// 注意这个参数，到时候还得再弄过来一下
	// mission_event;

	return SUCCESS;

}// __udp_socket_datalink::Send_Subscribe

