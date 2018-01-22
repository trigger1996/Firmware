#ifndef __Lidar_Drv_H
#define __Lidar_Drv_H

#include <rpi_config.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rplidar.h>


using namespace cv;
using namespace rp::standalone::rplidar;


typedef struct {
	_u8   Quality;
	float Angle;
	float Dst;
	float Vlct;		// 速度变量，这里本来该用联合体的，后面为了偷懒直接用这样表示

} __scandot;


class __lidar_driver
{
public:
	__lidar_driver();
	~__lidar_driver();

	Mat raw, raw_last;

	int init();
	int draw_Frames(int yaw);

private:

	const char *opt_com_path = "/dev/ttyUSB0";
	const _u32  opt_com_baudrate = 115200;
	RPlidarDriver *drv;

	// 原始数据
	vector<__scandot> Data;
	float Scan_Speed;

	// 归一化数据
	float Data_NArray[ANGLE_ALL];		// 归一化后的数据
	float Data_NLast[ANGLE_ALL];		// 归一化后数据备份


	bool check_RPLIDARHealth();
	int scan_Data();


	int normalize_Data(vector<__scandot> data);
	int normalize_Orentation(float data[], int yaw);

	int draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show);
	int draw(Mat &dst, float data[], char window_name[], bool is_show);	

};


#endif // __Lidar_Drv_H

