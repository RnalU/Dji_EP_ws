#pragma once

#include <string>

#ifndef PI
    #define PI     3.141592653   
#endif

#ifndef R2A
    #define R2A     180.0/PI
#endif

#ifndef A2R
    #define A2R     PI/180.0
#endif

#ifndef EARTH_RADIUS
    #define EARTH_RADIUS    6371393.0
#endif

//定义84坐标系
typedef struct str_wgs84
{
    double latitude;
    double longitude;
    float altitude;
}wgs84_t;

//定义笛卡尔直角坐标,右前上
typedef struct str_vector_xyz
{
    float x;
    float y;
    float z;
}vector_xyz_t;

//定义球极坐标,极坐标的基础直角坐标系为右前上
//https://baike.baidu.com/item/%E7%90%83%E6%9E%81%E5%9D%90%E6%A0%87%E7%B3%BB/6097240?fr=aladdin
typedef struct str_spc
{
    float r;
    float phi;
    float theta;
}spc_t;


class public_utils
{
public:
    public_utils();
    ~public_utils();

public:
    float GetTarWgs84Yaw(float curWgs84Yaw, float curRawYaw);
    float GetTarAngleByX(float curX, float curY);

public:
    wgs84_t RadarRaw2wgs84(vector_xyz_t raw_xyz, wgs84_t base_wgs84, float yaw, float irradiation_angle);
    vector_xyz_t wgs84toXYZ(wgs84_t base_wgs84, wgs84_t tag_wgs84, float yaw);
    spc_t wgs84toSpc(wgs84_t base_wgs84, wgs84_t tag_wgs84, float yaw);
    vector_xyz_t spc2xyz(spc_t rpt);
    vector_xyz_t ned2xyz(vector_xyz_t ned, float yaw, float irradiation_angle);
    float GetDisSpc(spc_t rpt1, spc_t rpt2);

    void CoordinateTransferUtil(wgs84_t  wgs84) ;
    double* wgs842polar(double lon, double lat, double height);
    wgs84_t polar2WGS84(spc_t  spc);

private:
    double latIterate(double initLat, double x, double y, double z);
    
};