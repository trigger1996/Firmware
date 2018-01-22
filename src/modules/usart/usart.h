#ifndef USART_H
#define USART_H



#include <termios.h>			// 这个是串口用的
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <nuttx_config.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_attitude.h>

#define READ_LEN_MAX		256
#define SEND_LEN_MAX		256

class __usart_report
{
public:
	__usart_report();

	bool thread_should_exit = false;		/**< daemon exit flag */
	bool thread_running = false;		/**< daemon status flag */
	int  daemon_task;				/**< Handle of daemon task / thread */


	char Cache[READ_LEN_MAX];

	void usage(const char *reason);

	/*
		* TELEM1 : /dev/ttyS1
		* TELEM2 : /dev/ttyS2
		* GPS : /dev/ttyS3
		* NSH : /dev/ttyS5
		* SERIAL4: /dev/ttyS6
		* N/A : /dev/ttyS4
		* IO DEBUG (RX only):/dev/ttyS0
	*/
	int init(const char * uart_name);
	int set_Baudrate(unsigned int baud);

	int read_nBytes(int nbytes);
	int send_nBytes(char data[], int nbytes);

private:

	int fd;

	struct termios uart_config;
	int termios_state;


};

#endif // USART_H

