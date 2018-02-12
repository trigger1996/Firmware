#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H


#include <rpi_config.h>
#include <rplidar.h>
#include <opencv2/opencv.hpp>

#include <lidar_common.h>

using namespace std;
using namespace cv;
using namespace rp::standalone::rplidar;

class __lidar_driver
{
public:
	__lidar_driver();					// 构造
	~__lidar_driver();					// 销毁

	vector<__scandot> Data;

	float Scan_Speed;

	Mat Img, Img_Last;					// 距离图，原始数据

	int init();

	int scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency, bool is_clear_old);

	int run();

	int draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show);


private:

	char		 *opt_com_path;
	_u32         opt_com_baudrate;
	RPlidarDriver * drv;

	bool check_RplidarHealth();


};



#endif // LIDAR_DRIVER_H
