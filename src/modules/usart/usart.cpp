#include "usart.h"

__usart_report::__usart_report()
{

}// __usart_report::__usart_report()

int __usart_report::set_Baudrate(unsigned int baud)
{
	int speed;

	switch (baud) {
		case 9600: speed = B9600; break;
		case 19200: speed = B19200; break;
		case 38400: speed = B38400; break;
		case 57600: speed = B57600; break;
		case 115200: speed = B115200; break;
		default:
			warnx("ERR: baudrate: %d\n", baud);
			return -EINVAL;
	}

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);
	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return false;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return false;
	}

	return true;
}// int __usart_report::set_baudrate(unsigned int baud)

int __usart_report::init(const char * uart_name)
{
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
		return false;
	}

	fd = serial_fd;
	return serial_fd;

}// int __usart_report::uart_init(const char * uart_name)

int __usart_report::read_nBytes(int nbytes)
{
	int i;
	char buffer[READ_LEN_MAX] = { 0 };

	if (nbytes > READ_LEN_MAX)
		nbytes = READ_LEN_MAX;

	read(fd, buffer, nbytes);

	for (i = 0; i < nbytes; i++)
	{
		Cache[i] = buffer[i];
	}

	return SUCCESS;

}// int __usart_report::read_nBytes(int nbytes)

int __usart_report::send_nBytes(char data[], int nbytes)
{
	int stat = SUCCESS;

	if (nbytes > SEND_LEN_MAX)
		nbytes = SEND_LEN_MAX;

	stat = write(fd, data, nbytes);

	return SUCCESS;

}// int __usart_report::send_nBytes(char data[], int nbytes)
