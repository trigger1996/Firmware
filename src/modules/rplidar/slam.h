#ifndef SLAM_H

#define SLAM_H

#include <rpi_config.h>

#include <sys/time.h>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <rotation_mat.h>

using namespace cv;

class __slam
{
public:

	__slam();
	~__slam();

	// 计算得到的速度和位置
	double x, y;
	double vx, vy;

	// 作出的图
	Mat Map;

	void update_Data(double vx_raw,   double vy_raw,
					 double acc_x_in,   double acc_y_in,
					 __AHRS ahrs_in);
	void update_Obstacle(float data[]);

	int  run();

private:

	double vx_in, vy_in;
	double acc_x, acc_y;
	__AHRS ahrs;

	/// 卡尔曼滤波器常数
	//  常量只能放在这里赋值
	const double F = 1;
	const double H = 1;
	const double I = 1;
	const double Q = 0.05  * 0.05;
	const double R = 0.005 * 0.005;



	/// 卡尔曼滤波器全局变量
	double Xx, Xy;
	double Px, Py;
	double Kx, Ky;

	// 地图障碍物
	float obstacle[ANGLE_ALL];

	// 地图上的位置
	const int x_pixel_bias = 900;
	const int y_pixel_bias = 900;		// 对图像上的坐标加个偏移，这样可以更好观察地图，不会一开始从左上角开始
	int x_pixel, y_pixel;

	// 更新标识位
	bool is_basicdata_updated;
	bool is_mapdata_updated;


	// 时间变量
	timeval t_now, t_last;
	double dt;

	void kalman_filter();
	void draw_Map(bool is_show);
	int  update_Time();

};


#endif // SLAM_H
