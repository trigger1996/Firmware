#include <slam.h>

__slam::__slam()
{
	// 卡尔曼滤波器全局变量
	Xx = 0, Xy = 0;
	Px = 1, Py = 1;
	Kx = 1, Ky = 1;

	x = y = 0;
	yaw = 0;

	x_pixel = y_pixel = Map_ImgSize / 2;

	Mat zero(Map_ImgHeight, Map_ImgWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	Map = zero.clone();
	zero.release();

	// 时间初始化
#ifndef WIN32

#else
	t_now = t_last = 0;
	dt = 0;
#endif

}// __slam::__slam()

__slam::__slam(double yaw_initial)
{
	__slam();

	// 初始化的时候不知道为什么前面写好了这里还没作用
	if (Map.empty())
	{
		Mat zero(Map_ImgHeight, Map_ImgWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
		Map = zero.clone();
		zero.release();
	}

	yaw = yaw_initial;

}// __slam::__slam(double yaw_initial)


__slam::~__slam()
{

}// __slam::~__slam()

int __slam::run(vector<__scandot> in, double dx_in, double dy_in, double d_yaw_in,
						double acc_x_in, double acc_y_in,
						__AHRS ahrs_in)
{
	int stat;

	update_Data(in, dx_in, dy_in, d_yaw_in);
	update_QuadData(acc_x_in, acc_y_in, ahrs_in);

	stat = run();

	return stat;

}// int __slam::run(vector<__scandot> in, double dx_in, double dy_in, double d_yaw_in)


int __slam::run()
{
	///
	/// 更新时间
#ifndef WIN32
	dt = (double)update_Time() / 1000;
#else
	dt = update_Time();
#endif

	///
	///
	dt = dt / 1000.0f;

	///
	/// 距离微元换速度
	vx_in = dx / dt; vy_in = dy / dt;					// ICP算法只能得到x和y的微元，并不是严格的速度，这里要换算速度

	///
	/// 调用卡尔曼滤波器
	kalman_filter();

	///
	/// 偏航角互补滤波
	//  其实只要图像畸变不是那么厉害，这些数据还是能用的
	const double ky1 = 0.85f;
	const double ky2 = 0.15f;
	yaw = yaw + d_yaw;								// 积分
																//  偏航角范围: -180~179
	while (yaw >= 2 * PI)							// 如果大于2*PI则要不停地减，直到减到2*PI以内
		yaw -= 2 * PI;
	while (yaw <= -2 * PI)							// 反之，如果小于则要不停的加，直到加到-2*PI以内
		yaw += 2 * PI;
	if (yaw >= PI)										// 如果大于PI(180度)，则要变成负的
		yaw -= 2 * PI;
	else if (yaw <= -PI)								// 如果小于PI，则要变成正的
		yaw += 2 * PI;
	// 互补滤波器
	yaw = ahrs.Yaw * PI / 180.0f * ky1 + yaw * ky2;		// 在仿真的时候这个要注释掉，不然会导致偏航角异常，因为根本没有实际的IMU输入

	///
	/// 速度机体->对地速度
	double vx_gnd = 0, vy_gnd = 0;
	double z_out_temp = 0;
	rotation_mat_inv(vx_in, vy_in, 0, ahrs.Roll, ahrs.Pitch, ahrs.Yaw, &vx_gnd, &vy_gnd, &z_out_temp);

	///
	/// 速度积分得到位移
	//x = x + dx;										// 积分
	//y = y + dy;										// 积分
	x = x + vy_gnd * dt;
	y = y + vx_gnd * dt;

	///
	/// 作图
	x_pixel =	y	* Map_ImgScale + x_pixel_bias;
	y_pixel =	-x	* Map_ImgScale + y_pixel_bias;

	draw_Map(true);

	PX4_INFO("acc_x: %f, acc_y: %f", vx_in, vy_in);
	PX4_INFO("x: %f, y: %f", x, y);
	return SUCCESS;

}// int __slam::run()

void __slam::kalman_filter()
{
	// 核心代码
	// 输入单位:		速度v:		mm/s
	//				加速度a:		mm/s^2
	//				角度:		deg
	//				时间:		s

	double a, b, c;
	double temp;

	/// 清零输出
	vx = vy = 0;

	/// 角度转弧度
	a = ahrs.Roll  * PI / 180.0f;			// Roll
	b = ahrs.Pitch * PI / 180.0f;		// Pitch
	c = ahrs.Yaw   * PI / 180.0f;		// Yaw

	/// 先归一化速度
	/// 为什么前面要锁定yaw，这里再换算，这是因为光流会受到旋转的影响，如果前面不锁定虽然可以避免下面的步骤，但是噪声会非常大
	/// 对调x, y速度，因为图像的x, y和实际相反
	//  改代码的时候记得放到外面
	temp = vx_in; vx_in = vy_in; vy_in = temp;

	/// 速度修正: 准对地速度->对地速度
	vx_in = vx_in * cos(b);				// 注意这边乘以的是轴向的速度
	vy_in = vy_in * cos(a);
	/// 对地速度->机体速度
	//vx_in = vx_in * (sin(a) * sin(c) - cos(a) * sin(b) * cos(c));
	//vy_in = vy_in * (sin(a) * cos(c) + cos(a) * sin(b) * sin(c));
	double z_out_temp = 0;
	rotation_mat(vx_in, vy_in, 0, ahrs.Roll, ahrs.Pitch, ahrs.Yaw, &vx_in, &vy_in, &z_out_temp);

	Xx = F * Xx + acc_x * dt;
	Px = F * Px + Q;
	Kx = H * Px / (H * Px + R);
	Xx = Xx + Kx * (vx_in - Xx * H);
	Px = (I - Kx) * Px;

	Xy = F * Xy + acc_y * dt;
	Py = F * Py + Q;
	Ky = H * Py / (H * Py + R);
	Xy = Xy + Ky * (vy_in - Xy * H);
	Py = (I - Ky) * Py;

	// 取出值，因为相对运动，所以这边要取负号
	vx = Xx;
	vy = Xy;

	// 测试代码，输出卡尔曼增益
	//PX4_INFO("Kalman Gain: Kx: %f, Ky: %f", Kx, Ky);

	// 首先看姿态角，如果倾斜的很厉害就不做计算了
	//if (ahrs.Pitch >= 5 * PI / 180.0f)
	//{
	//	vx_dst = vy_dst = 0;
	//}


}// void __slam::kalman_filter()

int __slam::update_Data(vector<__scandot> in, double dx_in, double dy_in, double d_yaw_in)
{
	int i;

	data.clear();
	for (i = 0; i < in.size(); i++)
	{
		__scandot temp;
		temp = in[i];
		data.push_back(temp);
	}

	dx = dy_in, dy = -dx_in;		// 图像的x, y刚好和机体坐标系的x, y相反
	d_yaw = d_yaw_in;

	return SUCCESS;

}// int __slam::update_Data(vector<__scandot> in, double dx_in, double dy_in, double d_yaw_in)

int __slam::update_QuadData(double acc_x_in, double acc_y_in,
							__AHRS ahrs_in)
{

	acc_x = acc_x_in;
	acc_y = acc_y_in;
	ahrs = ahrs_in;

	acc_x = -acc_x * 1000.0f;
	acc_y = acc_y  * 1000.0f;

	//PX4_INFO("SLAM: Pitch: %f, Roll: %f, Yaw: %f", ahrs.Pitch, ahrs.Roll, ahrs.Yaw);

	return SUCCESS;

}// void __slam::update_Data(..)

void __slam::draw_Map(bool is_show)
{
	double x_pt, y_pt;
	double x_rot, y_rot;
	double theta, rho;

	// 测试代码
	//Mat zero(Map_ImgHeight, Map_ImgWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	//Map = zero.clone();
	//zero.release();
	// 当前位置
	circle(Map, Point((int)x_pixel, (int)y_pixel), 2, Scalar(255, 0, 255), -1, 8, 0);


	for (unsigned int i = 0; i < data.size(); i++)	// scan_data.size()
	{
		__scandot dot;
		dot = data[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

		rho = dot.Dst;
		theta = dot.Angle * PI / 180;

		x_pt = rho   * sin(theta) ;
		y_pt = -rho * cos(theta) ;
		// 角度旋转处理，用一个旋转矩阵
		x_rot = x_pt *	cos(-yaw)	+ y_pt * sin(-yaw);		// yaw已经是弧度了
		y_rot = x_pt *	-sin(-yaw) + y_pt * cos(-yaw);
		x_rot = x_rot * Map_ImgScale + x_pixel;
		y_rot = y_rot * Map_ImgScale + y_pixel;

		circle(Map, Point((int)x_rot, (int)y_rot), 1, Scalar(0, 125, 255), -1, 8, 0);

	}
	if (is_show)
		imshow("Slam Map", Map);

}// void __slam::draw_Map()

// 更新时间
#ifndef WIN32
double __slam::update_Time()
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
#else
double __slam::update_Time()
{
	// http://blog.csdn.net/xpplearnc/article/details/53894048
	// Windows C下如何获得毫秒/微秒级时间
	double t = 0;

	t_last = t_now;
	t_now = (double)clock();
	t = t_now - t_last;

	return t;

}// float __slam::update_Time()
#endif
