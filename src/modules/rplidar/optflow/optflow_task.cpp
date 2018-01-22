#include "optflow.h"

// 这么写是为了防止程序耦合
using namespace std;
using namespace cv;


// 这个函数是Farneback的复制版本
// 计划对其进行简化

// 参数说明如下：
// _prev0：输入前一帧图像
// _next0：输入后一帧图像
// _flow0：输出的光流
// pyr_scale：金字塔上下两层之间的尺度关系
// levels：金字塔层数
// winsize：均值窗口大小，越大越能denoise并且能够检测快速移动目标，但会引起模糊运动区域
// iterations：迭代次数
// poly_n：像素领域大小，一般为5，7等
// poly_sigma：高斯标注差，一般为1-1.5
// flags：计算方法。主要包括OPTFLOW_USE_INITIAL_FLOW和OPTFLOW_FARNEBACK_GAUSSIAN
void calcOptFlow_Dense(const Mat& prev0, const Mat& next0,
	Mat& flow0, double pyr_scale, int levels, int winsize,
	int iterations, int poly_n, double poly_sigma, int flags);

int __optflow::run(Mat raw, Mat raw_last, bool is_show)
{
	Mat prevgray, gray, flow, cflow, frame;

	resize(raw_last, prevgray, Size(150, 150), (0, 0), (0, 0), cv::INTER_LINEAR);
	resize(raw, gray, Size(150, 150), (0, 0), (0, 0), cv::INTER_LINEAR);

	cvtColor(prevgray, prevgray, CV_RGB2GRAY);
	cvtColor(gray, gray, CV_RGB2GRAY);
	if (prevgray.data)
	{
		// 测试代码
		//warnx("calculating optflow...");

		// 这边不用标准的FarneBack，改用自己的代码
		calcOptFlow_Dense(prevgray, gray, flow, 0.5, 2, 15, 3, 5, 1.2, 1);		// prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0
		cvtColor(prevgray, cflow, CV_GRAY2RGB);

		// 归零数据，更新数据，准备滑动平均值滤波
		vx_now = vx_pre = vx_last = 0;
		vy_now = vy_pre = vy_last = 0;

		int pt_eff_x, pt_eff_y;
		vx = vy = 0;
		pt_eff_x = pt_eff_y = 0;
		for (int y = 0; y < cflow.rows; y += 5)
		{
			for (int x = 0; x < cflow.cols; x += 5)
			{
				Point2f fxy = flow.at<Point2f>(y, x);
				line(cflow, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), CV_RGB(0, 255, 125));
				circle(cflow, Point(x, y), 2, CV_RGB(125, 125, 0), -1);

				if (fabs(fxy.x) >= 0.01f)
				{
					vx_now += fxy.x;
					pt_eff_x++;
				}
				if (fabs(fxy.y) >= 0.01f)
				{
					vy_now += fxy.y;
					pt_eff_y++;
				}

			}
		}
		if (is_show)
			imshow("FLOW", cflow);

		if (pt_eff_x != 0)
		{
			vx_now /= pt_eff_x;
			vx_now = vx_now * 4 / LidarImageScale;		// 比例尺换算

		}
		if (pt_eff_y != 0)
		{
			vy_now /= pt_eff_y;
			vy_now = vy_now * 4 / LidarImageScale;		// 比例尺换算
		}

		//cout << "Vx: " << vx << " " << "Vy: " << vy << endl;
		PX4_INFO("Vx: %5.5f, Vy: %5.5f \n", vx_now, vy_now);

		x += vx_now;
		y += vy_now;
		//cout << "X: " << x << " " << "Y: " << y << endl << endl;
		PX4_INFO("X: %5.5f, Y: %5.5f \n", x, y);
	}

	return SUCCESS;

}// int __optflow::run()

__optflow::__optflow()
{
	x  = y  = 0.0f;
	vx = vy = 0.0f;

	vx_now = vx_pre = vx_last = 0.0f;
	vy_now = vy_pre = vy_last = 0.0f;

}// __optflow::__optflow()

__optflow::~__optflow()
{


}// __optflow::~__optflow()
