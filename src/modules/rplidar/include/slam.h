#ifndef __SLAM_H

#define __SLAM_H

#include <lidar_common.h>
#include <rpi_config.h>
#include <rotation_mat.h>
#include <opencv2/opencv.hpp>

using namespace cv;

#define Map_ImgSize			(LidarImageSize * 3)
#define Map_ImgWidth		Map_ImgSize
#define Map_ImgHeight		Map_ImgSize
#define Map_ImgScale		LidarImageScale
#ifndef WIN32

#else
	#include <Windows.h>
	#include <time.h>
#endif



class __slam
{
public:

	__slam();
	__slam(double yaw_initial);
	~__slam();

	double x, y;
	double vx, vy;
	double yaw;
	Mat    Map;

	int update_Data(vector<__scandot> in,
					double dx_in, double dy_in,
					double d_yaw_in);
	int update_QuadData(double acc_x_in, double acc_y_in,
						__AHRS ahrs_in);

	int run();
	int run(vector<__scandot> in, double dx_in, double dy_in, double d_yaw_in,
			double acc_x_in, double acc_y_in,
			__AHRS ahrs_in);

private:

	// icp输入
	vector<__scandot> data;
	double dx, dy;
	double d_yaw;

	// 机体状态输入
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

	// 地图上的位置
	const int x_pixel_bias = Map_ImgWidth  / 2;
	const int y_pixel_bias = Map_ImgHeight / 2;		// 对图像上的坐标加个偏移，这样可以更好观察地图，不会一开始从左上角开始
	int x_pixel, y_pixel;

	void kalman_filter();

	void draw_Map(bool is_show);

	// 更新时间，这里规定: 统一使用毫秒级的时间
	// 因为Linux和Windows的获取时间手段不同，所以注意一下
#ifndef WIN32
	timeval t_now, t_last;
#else
	double t_last, t_now;
#endif
	double dt;
	double update_Time();
};




#endif	/* __SLAM_H */
