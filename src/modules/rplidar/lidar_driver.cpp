#include "lidar_common.h"
#include "lidar_driver.h"

__lidar_driver::__lidar_driver()
{
	drv = NULL;

}// __lidar_driver::__lidar_driver()

__lidar_driver::~__lidar_driver()
{

}// __lidar_driver::~__lidar_driver()

int __lidar_driver::draw_Frames(int yaw)
{

	scan_Data();

	normalize_Data(Data);

	normalize_Orentation(Data_NArray, yaw);

	// 预处理结果
	draw(raw, Data_NArray, (char *)"raw", false);
	draw(raw_last, Data_NLast, (char *)"raw_last", false);

	return SUCCESS;

}// int __lidar_driver::draw_Frames()

int __lidar_driver::init()
{
	//u_result     op_result;

	// 开驱动
	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		PX4_ERR("[Rplidar] insufficent memory, exit\n");
		return FAILED;
	}

	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		PX4_ERR("Error, cannot bind to the specified serial port %s.\n");

		return FAILED;
	}

	// check health...
	if (!check_RPLIDARHealth()) {

		PX4_ERR("[Rplidar] RpLidar unhealthy, exit\n");
		return FAILED;
	}

	// start scan...
	drv->startMotor();
	drv->startScan();

	return SUCCESS;

}// int __lidar_driver::init()

bool __lidar_driver::check_RPLIDARHealth()
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.

		warnx("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {

			PX4_ERR("Error, rplidar internal error detected. Please reboot the device to retry.\n");
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
}// bool __lidar_driver::checkRPLIDARHealth()

int __lidar_driver::scan_Data()
{
	rplidar_response_measurement_node_t nodes[360 * 2];
	size_t   count = _countof(nodes);

	u_result op_result;
	bool is_express = false;
	bool is_4k_mode = false;

	op_result = drv->grabScanData(nodes, count);

	if (IS_OK(op_result)) {
		float frequency = 0;

		drv->getFrequency(is_express, count, frequency, is_4k_mode);

		//将扫面的原始数据转化为实际距离和角度，这个可以参考ultra_simple
		if (count == 0) return FAILED;

		// 写入新数据
		Data.clear();
		for (int pos = 0; pos < (int)count; ++pos) {
			__scandot dot;
			if (!nodes[pos].distance_q2) continue;

			dot.Quality = (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			dot.Angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			dot.Dst = nodes[pos].distance_q2 / 4.0f;
			Data.push_back(dot);
		}

		Scan_Speed = frequency;
	}

	return SUCCESS;

}// int __lidar_driver::scan_Data()

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

		circle(dst, Point(x, y), 2, Scalar(0, 0, 255), -1, 8, 0);

	}

	//char s[35];
	//sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
	//putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

	if (is_show)
		imshow(window_name, dst);

	return SUCCESS;

}// int __lidar_driver::draw(Mat dst, char window_name[])

int __lidar_driver::draw(Mat &dst, float data[], char window_name[], bool is_show)
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


	for (unsigned int i = 0; i < ANGLE_ALL; i++)	// scan_data.size()
	{

		theta = i * PI / 180;
		rho = data[i];

		x = (int)(rho  * sin(theta) * LidarImageScale) + halfWidth;
		y = (int)(-rho * cos(theta) * LidarImageScale) + halfHeight;

		circle(dst, Point(x, y), 2, Scalar(0, 0, 255), -1, 8, 0);

	}

	//char s[35];
	//sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
	//putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

	if (is_show)
		imshow(window_name, dst);

	return SUCCESS;

}// int __lidar_driver::draw(Mat dst, char window_name[])

int __lidar_driver::normalize_Data(vector<__scandot> data)
{
	int i;
	bool has_upper[360] = { false };
	bool has_lower[360] = { false };

	// 备份数据
	memset(Data_NLast, 0.0f, ANGLE_ALL * sizeof(float));
	for (i = 0; i < ANGLE_ALL; i++)
		Data_NLast[i] = Data_NArray[i];


	memset(Data_NArray, 0.0f, ANGLE_ALL * sizeof(float));
	for (i = 0; i < data.size() - 1; i++)
	{

		float angle   = data[i].Angle;
		int angle_neg = (int)data[i].Angle;
		int angle_pos = (int)data[i].Angle + 1;
		//float dst = data[i].Dst;

		// TODO 把角度向两边分解
		Data_NArray[angle_neg] = data[i].Dst * cos((angle - angle_neg) * PI / 180.0f);
		Data_NArray[angle_pos] = data[i].Dst * cos((angle_pos - angle) * PI / 180.0f);

	}

	// 测试代码
	//warnx("dot number: %d\n", i);

	return SUCCESS;


}// int __lidar_driver::normalize_Data(vector<__scandot> data)

int __lidar_driver::normalize_Orentation(float data[], int yaw)
{
	// 根据当前偏航方向旋转数据
	int i;
	float temp[ANGLE_ALL] = { 0 };

	// 归一化角度
	yaw = (yaw + 360) % 360;

	// 测试代码
	//warnx("yaw: %f", yaw);

	// 先把数据复制一份出去
	for (i = 0; i < ANGLE_ALL; i++)
	{
		temp[i] = data[i];
	}

	// 进行旋转
	for (i = 0; i < ANGLE_ALL; i++)
	{
		data[(i + yaw) % ANGLE_ALL] = temp[i];
	}

	return SUCCESS;


}// int __lidar_driver::normalize_Orentation(int yaw)

