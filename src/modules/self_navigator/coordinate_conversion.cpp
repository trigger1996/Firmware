#include "self_navigator.h"
#include "coordinate_conversion.h"


__coordinate_cov::__coordinate_cov()
{
	int i;

	Home        = { 0, 0, 0 };
	current_Pos = { 0, 0, 0 };
	last_Pos    = { 0, 0, 0 };
	next_Pos    = { 0, 0, 0 };

	for (i = 0; i < MAX_NAV_PT - 1; i++)
		Nav_Pt[i] = { 0, 0, 0 };

}// __coordinate_cov::__coordinate_cov()


int __coordinate_cov::update_Current_Pos(int lat, int lng, float alt)
{
	// 经纬度乘以10^7

	last_Pos = current_Pos;
	current_Pos.Lat = lat;
	current_Pos.Lng = lng;
	current_Pos.Alt = (int)(alt * 1000.0f);


	return SUCCESS;

}// int __coordinate_cov::update_Current_Pos(int lat, int lng, int alt)

int __coordinate_cov::update_HomePos(double lat, double lng, float alt)
{

	Home.Lat = (int)(lat * 10000000.0f);
	Home.Lng = (int)(lng * 10000000.0f);
	Home.Alt = (int)(alt * 1000.0f);

	warnx("Got HomePos, Lat: %d, Lng: %d", Home.Lat, Home.Lng);

	return SUCCESS;
}// int __coordinate_cov::Update_HomePos(int lat, int lng, int alt)

__NED __coordinate_cov::WGS84toNED(int lat_tgt, int lng_tgt, int alt_tgt)
{
	// 注意z方向向上为负
	// 这边运算精度低了很容易出问题
	double d_lat;
	double d_lng;

	next_Pos.Lat = lat_tgt;
	next_Pos.Lng = lng_tgt;
	next_Pos.Alt = alt_tgt;

	d_lat = (double)lat_tgt - (double)Home.Lat;
	d_lng = (double)lng_tgt - (double)Home.Lng;

	d_lat /= (double)10000000.0f;
	d_lng /= (double)10000000.0f;


	// x表示n,y表示e,z表示d
	double temp;
	__NED ret;

	// 纬度
	double lat_avg = (double)(Home.Lat + lat_tgt) / 2 / 10000000.0f;
	temp  = d_lat * 2 * MATH_PI * EARTH_RADIUS * cos(lat_avg) / 360.0f * 2.0f;	// 纬度只有180度
	ret.X = (float)temp;

	// 经度
	temp  = d_lng * 2 * MATH_PI * EARTH_RADIUS / 360.0f;
	ret.Y = (float)temp;

	// 高度
	ret.Z = -(float)alt_tgt / 1000.0f;										// 这边输入的alt都是正值

	return ret;


}// __grid __coordinate_cov::WGS84toNED(int lat_tgt, int lng_tgt, int alt_tgt)

__NED __coordinate_cov::WGS84toNED(double lon1, double lat1,double lon2, double lat2, float alt_tgt)
{
	/**
	 *  注意，lon1, lat1表示home_pos
	 *
	*/

	/**
	 * 基于余弦定理求两经纬度距离
	 * @param lon1 第一点的精度
	 * @param lat1 第一点的纬度
	 * @param lon2 第二点的精度
	 * @param lat3 第二点的纬度
	 * @return 返回的距离，单位km
	 * */

	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);

	double radLon1 = rad(lon1);
	double radLon2 = rad(lon2);

	if (radLat1 < 0)
		radLat1 = MATH_PI / 2 + fabs(radLat1);// south
	if (radLat1 > 0)
		radLat1 = MATH_PI / 2 - fabs(radLat1);// north
	if (radLon1 < 0)
		radLon1 = MATH_PI * 2 - fabs(radLon1);// west
	if (radLat2 < 0)
		radLat2 = MATH_PI / 2 + fabs(radLat2);// south
	if (radLat2 > 0)
		radLat2 = MATH_PI / 2 - fabs(radLat2);// north
	if (radLon2 < 0)
		radLon2 = MATH_PI * 2 - fabs(radLon2);// west

	double x1 = EARTH_RADIUS * cos(radLon1) * sin(radLat1);
	double y1 = EARTH_RADIUS * sin(radLon1) * sin(radLat1);
	double z1 = EARTH_RADIUS * cos(radLat1);

	double x2 = EARTH_RADIUS * cos(radLon2) * sin(radLat2);
	double y2 = EARTH_RADIUS * sin(radLon2) * sin(radLat2);
	double z2 = EARTH_RADIUS * cos(radLat2);

	double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)+ (z1 - z2) * (z1 - z2));
	//余弦定理求夹角
	double theta = acos((EARTH_RADIUS * EARTH_RADIUS + EARTH_RADIUS * EARTH_RADIUS - d * d) / (2 * EARTH_RADIUS * EARTH_RADIUS));
	double dist = theta * EARTH_RADIUS;

	// 这个函数本来是设计用来求距离的
	// 这边改一下变成求NED的
	__NED ret;
	ret.X = x2 - x1;
	ret.Y = y2 - y1;
	//ret.Z = z2 - z1;
	ret.Z = -alt_tgt;

	return ret;

}// __NED __coordinate_cov::WGS84toNED(double lon1, double lat1,double lon2, double lat2)

/**
 * 转化为弧度(rad)
 * */
double __coordinate_cov::rad(double d)
{

   return d * MATH_PI / 180.0;

}// double __coordinate_cov::rad(double d)
