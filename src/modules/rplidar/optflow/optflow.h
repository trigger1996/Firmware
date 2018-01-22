#pragma once


#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <rpi_config.h>
#include <opencv2/opencv.hpp>

using namespace cv;
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wunused-value"

class __optflow
{
public:

	__optflow();
	~__optflow();

	double vx, vy;
	//double x,  y;

	int run(Mat raw, Mat raw_last, bool is_show);

private:

	double vx_now, vx_pre, vx_last;
	double vy_now, vy_pre, vy_last;


};
