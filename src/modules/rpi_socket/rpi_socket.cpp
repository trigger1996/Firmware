#include "rpi_socket.h"

extern "C" __EXPORT int rpi_socket_main(int argc, char *argv[]);

__socket_thread SkT;
static int Flight_ID = 0x01;


int socket_thread_main(int argc, char *argv[])
{
	__socket_rpi Socket_Rpi(SERVER_IP, SERVER_PORT);	/// IP改为广播，地址改为50000 + ID，但是这里的端口是对方的端口，所以还是50000吧
	__udp_socket_datalink UDP_Link(Flight_ID);

	static int stat = FAILED;

	strcpy(UDP_Link.Tx, "Radio Check!\n");		// 不这么发因为存储区是连续的，所以会发一堆乱七八糟的出去
	Socket_Rpi.Send_Frame(UDP_Link.Tx);			// 一定Client先发一次才能正常工作
	memset(UDP_Link.Tx, 0, sizeof(UDP_Link.Tx));

    warnx("[rpi_socket] starting\n");

    SkT.thread_running = true;

    while (!SkT.thread_should_exit)
    {

		//UDP_Link.Send_Config();
		UDP_Link.Send_PlainText_Cfg_Preload();
		Socket_Rpi.Send_Frame(UDP_Link.Tx);

		stat = Socket_Rpi.Receive_Frame();                  // 如果主机不发就会一直卡在这里
		//warnx("[rpi_socket]: %s", Socket_Rpi.Rx);
		if (stat == SUCCESS)
		{
			UDP_Link.refine_Data_PlainText(Socket_Rpi.Rx);
			UDP_Link.Recv_to_ORB(UDP_Link.Server_Time,
								 UDP_Link.Nav_Pt[0].Lat, UDP_Link.Nav_Pt[0].Lng,
								 UDP_Link.Nav_Pt[0].Alt,
								 UDP_Link.Cmd_Extra);
			memset(Socket_Rpi.Rx, 0, sizeof(char) * Rx_Len_Max);

			/// 测试代码
			//warnx("[received] Lat: %d Lng: %d Alt: %d",
			//	  UDP_Link.Nav_Pt[0].Lat, UDP_Link.Nav_Pt[0].Lng,
			//	  UDP_Link.Nav_Pt[0].Alt);
		}

		usleep(400000);     // 50ms
        //sleep(1);
	}

    warnx("[rpi_socket] exiting.\n");

    SkT.thread_running = false;

    return 0;
}// socket_thread_main


__socket_rpi::__socket_rpi(const char server_ip[], const uint16_t port)
{
    // 创建套接字
    fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        warnx("Socket NOT Created \n");
        return;
    }
    bzero(&ServAddr, sizeof(ServAddr));

    ServAddr.sin_family = AF_INET;
    ServAddr.sin_addr.s_addr = inet_addr(server_ip);
    ServAddr.sin_port = htons(port);

	bzero(&MyAddr, sizeof(MyAddr));
	MyAddr.sin_family = AF_INET;
	MyAddr.sin_port   = htons(SERVER_PORT + Flight_ID);

	bind(fd, (struct sockaddr *)&MyAddr, sizeof(MyAddr));


	/// 新增加，设置超时
	timeout.tv_sec  = 2;
	timeout.tv_usec = 0;
	//setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(char*)&timeout,sizeof(timeout));
	setsockopt(fd, SOL_SOCKET,SO_RCVTIMEO, (char*)&timeout, sizeof(struct timeval));

    memset(Tx, 0, sizeof(char) * Tx_Len_Max);
    memset(Rx, 0, sizeof(char) * Rx_Len_Max);

}//__socket__rpi::__socket_rpi

char __socket_rpi::Send_Frame(char str[])
{
    // 发送数据
    int i;

    //if (strlen(str) > Tx_Len_Max) return FAILED;

    for (i = 0; i < Tx_Len_Max; i++)
        Tx[i] = str[i];

	//warnx("[send] %s", Tx);
	sendto(fd, Tx, Tx_Len_Max,0, (struct sockaddr *)&ServAddr,sizeof(ServAddr));

    return SUCCESS;

}//__socket_rpi::Receive_Frame

char __socket_rpi::Receive_Frame(void)
{
    // 接收数据
    int recv_result;

    memset(Rx, 0, Rx_Len_Max);
	len = sizeof(ServAddr);
	recv_result = recvfrom(fd, Rx, Rx_Len_Max, 0, (struct sockaddr*)&ServAddr, &len);	//recvfrom是拥塞函数，没有数据就一直拥塞

	if(recv_result == -1)
    {
		warnx("[rpi_socket_recv]recieve data fail!");
        return FAILED;
    }

    /* 测试代码 */
    //printf("client:%s\n",Rx);  //打印client发过来的信息
    //memset(Rx, 0, Rx_Len_Max);
    //sprintf(Rx, "I have recieved %d bytes data!\n", recv_result);  //回复client
    //printf("server:%s\n",Rx);  //打印自己发送的信息给
    //sendto(fd, Rx, Rx_Len_Max, 0, (struct sockaddr*)&ServAddr, len);  //发送信息给client，注意使用了clent_addr结构体指针

    return SUCCESS;

}//__socket_rpi::Receive_Frame


int rpi_socket_main(int argc, char *argv[])
{
	if (argc < 4) {						/// argc < 2
        SkT.usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (SkT.thread_running) {
            warnx("rpi_socket already running\n");
            /* this is not an error */
            return 0;
        }

        SkT.thread_should_exit = false;
        SkT.daemon_task = px4_task_spawn_cmd("rpi_socket",
                            SCHED_DEFAULT,
                            SCHED_PRIORITY_DEFAULT,
                            2000,
                            socket_thread_main,
                            (argv) ? (char *const *)&argv[2] : (char *const *)NULL);

		if (!strcmp(argv[2], "-p"))
		{
			Flight_ID = atoi(argv[3]);
			if (Flight_ID <= 0x01)
				Flight_ID  = 0x01;

			warnx("ID: Flight_ID: %d", Flight_ID);
		}

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        SkT.thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (SkT.thread_running) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }

    SkT.usage("unrecognized command");
    return 1;
}// rpi_socket_main

void __socket_thread::usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

	warnx("usage: rpi_socket {start|stop|status} [-p <ID Number>]");
	warnx("e.g.: rpi_socket start -p 1");

}// __socket_thread::usage

