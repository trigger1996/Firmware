#ifndef COORDINATE_CONVERSION_H
#define COORDINATE_CONVERSION_H

#include "rpi_config.h"


#define MAX_NAV_PT 6

class __coordinate_cov
{
public:

	__coordinate_cov();

	__grid Home;													// 起飞位置
	__grid last_Pos;
	__grid current_Pos;												// 当前坐标
	__grid next_Pos;												// 当前目标
	__grid Nav_Pt[MAX_NAV_PT - 1];									// 接下里几个航点

	int update_Current_Pos(int lat, int lng, float alt);			// 经纬度乘以10^7
	int update_HomePos(double lat, double lng, float alt);

	__NED WGS84toNED(int lat_tgt, int lng_tgt, int alt_tgt);		// GPS绝对坐标换算至本地坐标
	__NED WGS84toNED(double lon1, double lat1,double lon2, double lat2, float alt_tgt);


private:

	double EARTH_RADIUS = 6378137;									//赤道半径(单位m)
	double MATH_PI =3.14159265358979323;


	double rad(double d);

};

#endif // COORDINATE_CONVERSION_H

