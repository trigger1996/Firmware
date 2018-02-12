#include <lidar_driver.h>


int __lidar_driver::run()
{
	const int is_ok_thereshold = 2;							// 1
															// 采样几次后处理，这个值可以理解为后续的样本点，样本点越多，后续的精度增加非常多， 但是样本点越多，实时性越差
															// 是时候买个好的激光雷达了
	int is_ok_times = 0;
	u_result  op_result = -1;

	rplidar_response_measurement_node_t nodes[360 * 2];
	size_t   count = _countof(nodes);
	bool is_express = false;
	bool is_4k_mode = false;

	Data.clear();
	do {
		op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result))
		{
			float frequency = 0;

			// 读取数据
			drv->getFrequency(is_express, count, frequency, is_4k_mode);
			scanData(nodes, count, frequency, false);

			is_ok_times++;
		}
	} while (is_ok_times < is_ok_thereshold);

	// 读取数据
	draw(Img, Data, (char *)"Raw", false);

	waitKey(30);
	return SUCCESS;

}// int __lidar_driver::run()

__lidar_driver::__lidar_driver()
{
	// 构造
	opt_com_path = NULL;
	opt_com_baudrate = 115200;


}// __lidar_driver::__lidar_driver()

__lidar_driver::~__lidar_driver()
{
	// 销毁
	Img.release();
	Img_Last.release();

}// __lidar_driver::~__lidar_driver()

int __lidar_driver::init()
{
	// create the driver instance
	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com3";
#else
		opt_com_path = (char *)"/dev/ttyUSB0";
#endif
	}

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}


	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);

		waitKey(0);
		return -3;
	}

	// check health...
	if (!check_RplidarHealth()) {

		waitKey(0);
		return -4;
	}

	// start scan...
	drv->startMotor();
	drv->startScan();

	return 0;

}// int __lidar_driver::init()

bool __lidar_driver::check_RplidarHealth()
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}// bool __lidar_driver::check_RplidarHealth()

 //将扫面的原始数据转化为实际距离和角度，这个可以参考ultra_simple
int __lidar_driver::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency, bool is_clear_old)
{

	if (count == 0) return -1;

	if (is_clear_old)
		Data.clear();

	// 然后写入新数据
	for (int pos = 0; pos < (int)count; ++pos) {
		__scandot dot;
		if (!buffer[pos].distance_q2) continue;

		dot.Quality = (buffer[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
		dot.Angle = (buffer[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
		dot.Dst = buffer[pos].distance_q2 / 4.0f;
		Data.push_back(dot);
	}

	Scan_Speed = frequency;

	return SUCCESS;
}// int __lidar_driver::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency, bool is_clear_old)

 //将扫描点映射到画布上
int __lidar_driver::draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show)
{
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(29, 230, 181));		// 图片格式： BGR
	dst = zero.clone();
	zero.release();

	//在中心加上一个圆心
	//circle(dst, Point(dst.cols / 2, dst.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

	int x, y;
	double theta, rho;
	int halfWidth = dst.cols / 2;
	int halfHeight = dst.rows / 2;


	for (unsigned int i = 0; i < data.size(); i++)	// scan_data.size()
	{
		__scandot dot;
		dot = data[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

		theta = dot.Angle * PI / 180;
		rho = dot.Dst;

		x = (int)(rho  * sin(theta) * LidarImageScale) + halfWidth;
		y = (int)(-rho * cos(theta) * LidarImageScale) + halfHeight;

		circle(dst, Point(x, y), 1, Scalar(0, 0, 255), -1, 8, 0);

	}

	//char s[35];
	//sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
	//putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

	if (is_show)
		imshow(window_name, dst);

	return SUCCESS;

}// int __lidar_driver::draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show)

