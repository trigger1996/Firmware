#ifndef __ICP_Interface_H

#define __ICP_Interface_H

#include <rpi_config.h>
#include <lidar_common.h>
#include <opencv2/opencv.hpp>

#include <icp.h>

using namespace cv;

class __icp
{
public:

	__icp();

	double dx, dy;
	double d_yaw;

	int get_Pts(vector<__scandot> in);
	int run(vector<__scandot> in, bool is_show);
	int run(bool is_show);

private:

	vector<CvPoint2D32f> data, data_last, data_shifted;

	float R[4], T[2];				// 旋转矩阵、平移矩阵
	//CvMat r, t;					// 这个改成临时变量

	float dx_temp[3], dy_temp[3];	// 滑动平均值滤波
	float dyaw_temp[3];

	int calc_dR_dYaw();
};


#endif	/* __ICP_Interface_H */
