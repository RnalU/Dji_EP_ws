#include "public_utils.h"
#include <math.h>

public_utils::public_utils(){

}

public_utils::~public_utils(){

}

//机体坐标系航向转大地坐标系航向
//curWgs84Yaw -- 无人机当前大地坐标系航向
//curRawYaw -- 机体坐标系航向
float public_utils::GetTarWgs84Yaw(float curWgs84Yaw, float curRawYaw)
{
    float tarYaw = 0.0;

    tarYaw = curWgs84Yaw + curRawYaw;

    if(tarYaw > 360)
    {
        tarYaw -= 360;
    }

    return tarYaw;
}

//获取目标相对角度
//curY -- 目标Y轴位置
//curX -- 目标所在x轴位置
float public_utils::GetTarAngleByX(float curX, float curY)
{
    float tarAngle = 0.0;

    tarAngle = atan(curX/curY);

    return tarAngle;
}


//坐标转换函数，把雷达坐标系转换为经纬度坐标
//输入量    vector_xyz_t raw_xyz:雷达输出XYZ坐标
//          wgs84_t base_wgs84:飞机当前WGS84坐标
//          float yaw:飞机当前的航向角
//          float irradiation_angle:雷达当前照射角度，前为0，右为正
//输出量：目标所在的经纬度坐标
wgs84_t public_utils::RadarRaw2wgs84(vector_xyz_t raw_xyz, wgs84_t base_wgs84, float yaw, float irradiation_angle)
{
    wgs84_t out;
    float l_north;      
    float l_east;

    float d_angle = (yaw+irradiation_angle)*A2R;

    l_north = raw_xyz.y*cos(d_angle)-raw_xyz.x*sin(d_angle);
    l_east = raw_xyz.y*sin(d_angle)+raw_xyz.x*cos(d_angle);

    out.latitude = base_wgs84.latitude+R2A*l_north/EARTH_RADIUS;
    out.longitude = base_wgs84.longitude+R2A*l_east/(EARTH_RADIUS*cos(out.latitude*A2R));
    out.altitude = base_wgs84.altitude+raw_xyz.z;

    return out;
}

//坐标转换函数，把目标wgs84转换为相对自己的直角坐标
//输出量    wgs84_t base_wgs84:自己的地球坐标
//         wgs84_t tag_wgs84:目标的地球坐标
//          float yaw:当前航向
//输出量：目标所在位置相对于自己的直角坐标
vector_xyz_t public_utils::wgs84toXYZ(wgs84_t base_wgs84, wgs84_t tag_wgs84, float yaw)
{
    vector_xyz_t out;
    float l_north;      
    float l_east;

    l_north = (float)((tag_wgs84.latitude-base_wgs84.latitude)*A2R*EARTH_RADIUS);
    l_east = (float)((tag_wgs84.longitude-base_wgs84.longitude)*A2R*EARTH_RADIUS*cosf(base_wgs84.latitude*A2R));


    out.x = l_east*cos(yaw*A2R)-l_north*sin(yaw*A2R);
    out.y = l_east*sin(yaw*A2R)+l_north*cos(yaw*A2R);
    out.z = tag_wgs84.altitude-tag_wgs84.altitude;

    return out;
}


//坐标转换函数，把目标wgs84转换为相对自己的球极坐标系
//输出量    wgs84_t base_wgs84:自己的地球坐标
//         wgs84_t tag_wgs84:目标的地球坐标
//          float yaw:当前航向
//输出量：目标所在位置相对于自己的球极坐标
spc_t public_utils::wgs84toSpc(wgs84_t base_wgs84, wgs84_t tag_wgs84, float yaw)
{
    spc_t out;
    vector_xyz_t temp_xyz;

    temp_xyz = wgs84toXYZ(base_wgs84,tag_wgs84,yaw);

    out.r = sqrt(temp_xyz.x*temp_xyz.x+temp_xyz.y*temp_xyz.y+temp_xyz.z*temp_xyz.z);
    out.phi = atan2(temp_xyz.y, temp_xyz.x)*R2A;
    out.theta = acos(temp_xyz.z/out.r)*R2A;

    return out;
}


//坐标转换函数，把目标球极坐标系转换为笛卡尔坐标系
vector_xyz_t public_utils::spc2xyz(spc_t rpt)
{
    vector_xyz_t out;
    out.x = rpt.r*sin(rpt.theta*A2R)*cos(rpt.phi*A2R);
    out.y = rpt.r*sin(rpt.theta*A2R)*sin(rpt.phi*A2R);
    out.z = rpt.r*cos(rpt.theta*A2R);
    
    return out;
}

//坐标转换函数，把NED速度，转换到XYZ速度
vector_xyz_t public_utils::ned2xyz(vector_xyz_t ned, float yaw, float irradiation_angle)
{
    vector_xyz_t out;
    float d_angle = (yaw+irradiation_angle)*A2R;
 
    out.x = ned.y*cos(d_angle*A2R)-ned.x*sin(d_angle*A2R);
    out.y = ned.y*sin(d_angle*A2R)+ned.x*cos(d_angle*A2R);
    out.z = -ned.z;
 
    return out;
}



float public_utils::GetDisSpc(spc_t rpt1, spc_t rpt2)
{
	float dis = 0;
	float r = (rpt1.r+rpt2.r)*0.5f;
	float d_phi = rpt1.phi-rpt2.phi;
	
	if(d_phi > 180)
		d_phi -= 360;
	else if (d_phi < 180)
		d_phi += 360;
	
	
	dis = r*d_phi*A2R;
	
	return fabsf(dis);
	
}


double e2 = 0.00669437999013;
double X0 = 0, Y0 = 0, Z0 = 0;

double tracetarN;//卯酉圆曲率半径
double sintracetarlon, costracetarlon, sintracetarlat, costracetarlat;
void public_utils::CoordinateTransferUtil(wgs84_t  wgs84) 
{
    double tracetarlon, tracetarlat, tracetarheight;

    tracetarlon = wgs84.longitude;
    tracetarlat = wgs84.latitude;
    tracetarheight = wgs84.altitude;
 
    sintracetarlon = sin(PI * tracetarlon / 180);
    costracetarlon = cos(PI * tracetarlon / 180);
    sintracetarlat = sin(PI * tracetarlat / 180);
    costracetarlat = cos(PI * tracetarlat / 180);
    tracetarN = EARTH_RADIUS / (sqrt(1 - e2 * pow(sintracetarlat, 2)));
 
     /// 探测中心tracetar坐标转换为大地直角坐标 
    X0 = (tracetarN + tracetarheight) * costracetarlat * costracetarlon;
    Y0 = (tracetarN + tracetarheight) * costracetarlat * sintracetarlon;
    Z0 = (tracetarN * (1 - e2) + tracetarheight) * sintracetarlat;
}
 
/// <summary>
/// WGS-84经纬度以及高度转换为以探测中心traceTar为中心的极坐标
/// 
///方法：1、将空间地理坐标、tracetar坐标转换为大地直角坐标
///      2、将空间地理坐标的大地直角坐标转换为以tracetar为坐标原点的的enu（东北天）坐标//站心直角坐标
///      3、将gps相对tracetar的的enu坐标转换为极坐标。   //站心极坐标
/// </summary>
/// <param name="lon">目标经度</param>
/// <param name="lat">目标纬度</param>
/// <param name="height">目标海拔高度</param>
/// <param name="jzb[0]">斜距</param>
/// <param name="jzb[1]">方位角</param>
/// <param name="jzb[2]">俯仰角</param>
 
double* public_utils::wgs842polar(double lon, double lat, double height)
{
    double jzb[3] = {0};
    double r,a,b;
    double sinlon = sin(PI * lon / 180);//经度sin 参数
    double coslon = cos(PI * lon / 180);
    double sinlat = sin(PI * lat / 180);//维度sin参数
    double coslat = cos(PI * lat / 180);
    double N = EARTH_RADIUS / (sqrt(1 - e2 * pow(sinlat, 2)));//卯酉圆曲率半径
 
    ///1、gps 转换为大地直角坐标
    double X = (N + height) * coslat * coslon;
    double Y = (N + height) * coslat * sinlon;
    double Z = (N * (1 - e2) + height) * sinlat;
 
 
    ///2、大地直角坐标转换为enu坐标 即站心直角坐标
    double x = -sintracetarlon * (X - X0) + costracetarlon * (Y - Y0);
    double y = -sintracetarlat * costracetarlon * (X - X0) - sintracetarlon * sintracetarlat * (Y - Y0) + costracetarlat * (Z - Z0);
    double z = costracetarlat * costracetarlon * (X - X0) + costracetarlat * sintracetarlon * (Y - Y0) + sintracetarlat * (Z - Z0);
 
    ///3、站心直角坐标转换为站心极坐标。
    r = sqrt(x * x + y * y + z * z);
    if (x > 0 && y > 0)
    {
        a = atan(x / y) * 180 / PI;
 
    }
    else if (x < 0 && y > 0)
    {
 
        a = 360 - atan(abs(x) / y) * 180 / PI;
    }
    else if (x < 0 && y < 0)
    {
        a = 180 + atan(abs(x) / abs(y)) * 180 / PI;
    }
    else if (x > 0 && y < 0)
    {
 
        a = 180 - atan(x / abs(y)) * 180 / PI;
    }
    else if (x == 0 && y > 0)
    {
 
        a = 0;
    }
    else if (x == 0 && y < 0)
    {
        a = 180;
    }
    else if (x > 0 && y == 0)
    {
        a = 90;
    }
    else
    {
        a = 270;
    }
    b = asin(z / r) * 180 / PI;

    jzb[0] = r;
    jzb[1] =a;
    jzb[2] =b;

    return jzb;
}
/**
极坐标（中心坐标）转换为经纬度高度
///1、站心极坐标转为站心直角坐标。
///2、站心直角坐标系 转换为 大地直角坐标系
///3、大地直角坐标转换为经纬度WGS-84
/// r—斜距， a—方位， b—俯仰角
**/
wgs84_t public_utils::polar2WGS84(spc_t  spc)
{
    wgs84_t jwgd = {0};
    double lon,lat,height;
    double r = spc.r;
    double a = spc.phi;
    double b = spc.theta;

      ///1、站心极坐标转为站心直角坐标。
    double x1 = r * cos(PI * b / 180) * sin(PI * a / 180);
    double y1 = r * cos(PI * b / 180) * cos(PI * a / 180);
    double z1 = r * sin(PI * b / 180);
      
      ///2、站心直角坐标系 转换为 大地直角坐标系
    double xe = -sintracetarlon * x1 - sintracetarlat * costracetarlon * y1 + costracetarlon * costracetarlat * z1 + X0;
    double yn = costracetarlon * x1 - sintracetarlat * sintracetarlon * y1 + costracetarlat * sintracetarlon * z1 + Y0;
    double zu = costracetarlat * y1 + sintracetarlat * z1 + Z0;
      ///3、大地直角坐标转换为经纬度WGS-84
 
    lon = atan(yn / xe) * 180 / PI;
    if (lon < 0)
        lon = 180 + lon;
    double initLat = atan(zu / (sqrt(xe * xe + yn * yn))) * 180 / PI;
    lat = latIterate(initLat, xe, yn, zu);
    height = sqrt(xe * xe + yn * yn) / cos(PI * lat / 180) - tracetarN;

    jwgd.longitude = lon;
    jwgd.latitude = lat;
    jwgd.altitude = height;

    return jwgd;
}
 
double public_utils::latIterate(double initLat, double x, double y, double z)
{
    int iterCount = 40;///迭代法求纬度的迭代次数。 

    double sinlat = 0;
    double N = 0;

    double finalLat = initLat;
    int i = 0;

    while (true)
    {
        if (i >= iterCount)
        {
            break;
        }
        
        sinlat = sin(PI * finalLat / 180);
        N = EARTH_RADIUS / (sqrt(1 - e2 * pow(sinlat, 2)));
        finalLat = atan((z + N * e2 * sinlat) / sqrt(x * x + y * y)) * 180 / PI;
        i++;
    }

    return finalLat;
}




