#include "slam.h"


__slam::__slam()
{
	// 卡尔曼滤波器全局变量
	Xx = 0, Xy = 0;
	Px = 1, Py = 1;
	Kx = 1, Ky = 1;

	// 初始化时间
	t_now = t_last = { 0 };
	dt = 0;

	// 初始化地图
	Mat zero(LidarImageHeight * 3, LidarImageWidth * 3, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	Map = zero.clone();
	zero.release();

	memset(obstacle, 0, sizeof(float) * ANGLE_ALL);

	x_pixel = y_pixel = 200;

	// 更新标识位
	is_basicdata_updated = false;
	is_mapdata_updated = false;

}// __slam::__slam()


__slam::~__slam()
{

}// __slam::~__slam()

void __slam::update_Data(double vx_raw,   double vy_raw,
						 double acc_x_in,   double acc_y_in,
						 __AHRS ahrs_in)
{
	vx_in = vx_raw;
	vy_in = vy_raw;

	acc_x = acc_x_in;
	acc_y = acc_y_in;
	ahrs  = ahrs_in;

	//PX4_INFO("SLAM: Pitch: %f, Roll: %f, Yaw: %f", ahrs.Pitch, ahrs.Roll, ahrs.Yaw);

	// 更新时间
	dt = (double)update_Time() / 1000.0f;

	is_basicdata_updated = true;

}// void __slam::update_Data(..)

void __slam::update_Obstacle(float data[])
{
	int i;

	for (i = 0; i < ANGLE_ALL; i++)
		obstacle[i] = data[i];

	is_mapdata_updated = true;

}// void __slam::update_Obstacle(double data[])

int  __slam::run()
{
	if (is_basicdata_updated == true)
	{
		kalman_filter();

		/// 积分得到位置
		//  角度转弧度
		double a = ahrs.Roll  * PI / 180.0f;		// Roll
		double b = ahrs.Pitch * PI / 180.0f;		// Pitch
		double c = ahrs.Yaw   * PI / 180.0f;		// Yaw

		// 速度机体->对地速度
		// 这边懒得想了，直接除就是反变换了
		// vx_gnd->对地北方速度
		// vy_gnd->对地东方速度
		double vx_gnd = vx_in * (sin(a) * sin(c) - cos(a) * sin(b) * cos(c));
		double vy_gnd = vy_in * (sin(a) * cos(c) + cos(a) * sin(b) * sin(c));
		// 这里的x, y不对，要对惯性系做个转换
		x = x + vy_gnd * dt;
		y = y + vx_gnd * dt;

		is_basicdata_updated = false;
	}

	if (is_mapdata_updated == true)
	{

		x_pixel = x * LidarImageScale + x_pixel_bias;
		y_pixel = y * LidarImageScale + y_pixel_bias;

		draw_Map(false);

		is_mapdata_updated = false;
	}

	return SUCCESS;

}// int  __slam::run()

void __slam::kalman_filter()
{
	// 核心代码
	// 输入单位:	速度v:	mm/s
	//			加速度a:	mm/s^2
	//			角度:	deg
	//			时间:	ms

	double a, b, c;
	double temp;

	/// 清零输出
	vx = vy = 0;

	/// 处理时间
	dt = dt / 1000;

	/// 角度转弧度
	a = ahrs.Roll  * PI / 180.0f;		// Roll
	b = ahrs.Pitch * PI / 180.0f;		// Pitch
	c = ahrs.Yaw   * PI / 180.0f;		// Yaw

	/// 先归一化速度
	/// 为什么前面要锁定yaw，这里再换算，这是因为光流会受到旋转的影响，如果前面不锁定虽然可以避免下面的步骤，但是噪声会非常大
	/// 对调x, y速度，因为图像的x, y和实际相反
	//  改代码的时候记得放到外面
	temp = vx_in; vx_in = vy_in; vy_in = temp;
	/// 对地速度->机体速度
	vx_in = vx_in * (sin(a) * sin(c) - cos(a) * sin(b) * cos(c));
	vy_in = vy_in * (sin(a) * cos(c) + cos(a) * sin(b) * sin(c));
	/// 速度修正
	vx_in = vx_in * cos(b);				// 注意这边乘以的是轴向的速度
	vy_in = vy_in * cos(a);

	Xx = F * Xx + acc_x * 1000 * dt;
	Px = F * Px + Q;
	Kx = H * Px / (H * Px + R);
	Xx = Xx + Kx * (vx_in - Xx * H);
	Px = (I - Kx) * Px;

	Xy = F * Xy + acc_y * 1000 * dt;
	Py = F * Py + Q;
	Ky = H * Py /  (H * Py + R);
	Xy = Xy + Ky * (vy_in - Xy * H);
	Py = (I - Ky) * Py;

	// 取出值，因为相对运动，所以这边要取负号
	vx = -Xx;
	vy = -Xy;

	// 测试代码，输出卡尔曼增益
	//PX4_INFO("Kalman Gain: Kx: %f, Ky: %f", Kx, Ky);

	// 首先看姿态角，如果倾斜的很厉害就不做计算了
	//if (ahrs.Pitch >= 5 * PI / 180.0f)
	//{
	//	vx_dst = vy_dst = 0;
	//}


}// void __slam::kalman_filter()

void __slam::draw_Map(bool is_show)
{
	int x_pt, y_pt;
	double theta, rho;
	//int halfWidth  = Map.cols / 2;
	//int halfHeight = Map.rows / 2;

	for (unsigned int i = 0; i < ANGLE_ALL; i++)	// scan_data.size()
	{

		theta = i * PI / 180;
		rho = obstacle[i];

		x_pt = (int)(rho  * sin(theta) * LidarImageScale) + x_pixel;
		y_pt = (int)(-rho * cos(theta) * LidarImageScale) + y_pixel;

		if ((x_pt > 0 && x_pt < Map.cols) &&
			(y_pt > 0 && y_pt < Map.rows))
			circle(Map, Point(x_pt, y_pt), 2, Scalar(0, 125, 255), -1, 8, 0);

	}

	if (is_show)
		imshow("Slam Map", Map);

}// void __slam::draw_Map()

int __slam::update_Time()
{
	// 获得时间
	// 返回值单位: us
	int t;

	t_last = t_now;
	gettimeofday(&t_now, NULL);
	t = t_now.tv_usec - t_last.tv_usec;
	if (t < 0)
		t = t_now.tv_usec + 1000000 - t_last.tv_usec;
	//PX4_INFO("t_now: %d", dt);

	return t;

}// int __slam::update_Time()

