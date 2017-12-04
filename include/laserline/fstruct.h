#ifndef _FSTRUCH_H_
#define _FSTRUCH_H_

#include <vector>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>


#define PI 3.1415926535898
#define t(a) a*PI/180 //角度弧度转换

#define threshold_a 6     //聚类倍数
#define distance_point(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))
#define max(a,b) a>b?a:b
#define min(a,b) a<b?a:b

//点信息

typedef struct _POINT
{
	double x;
	double y;
}POINT;

typedef struct _CSData
{
	std::vector<unsigned int> index;//索引值
	std::vector<double> bearings;//角度
	std::vector<double> cos_value;//余弦
	std::vector<double> sin_value;//正弦
}CSdata;

typedef struct _RangeData
{
	std::vector<double> ranges;//role数值
	std::vector<double> xs;//x坐标
	std::vector<double> ys;//y坐标
}Rangedata;

//参数，从launch文件中读入
typedef struct _Params
{
	double angle_increment;//角度增量
	double angle_start;//初始角度
	double least_thresh;//正交拟合阈值
	double min_line_length;//拟合线段最短距离
	double predict_distance;//真实点与与预测点之间的距离阈值
	unsigned int min_line_points;//一条线段包含的激光点个数
	unsigned int seed_line_points;//种子线段包含的激光点个数
}Params;


typedef struct _word_params
{
	double _role;
	double _theta_one;
	double _theta_two;
}word_params;

typedef struct _signal_params
{
	double distance_signal;
}signal_params;

//直线段信息结构体
typedef struct _line
{
	double a;//直线参数
	double b;
	double c;
	int left;//直线范围
	int right;
	POINT p1;
	POINT p2;
	bool inte[2];
}line;

//直线方程式结构体
typedef struct _least
{
	double a;
	double b;
	double c;
}least;

//}

typedef struct _point
{
	double role;
  	double theta;
	double m_x;
  	double m_y;
  	double distance;
	double m_gradient;
	bool flag;
}PoinT;
	

typedef struct _generate_line
{
	//first point
	double x1;
	double y1;
	//end point
	double x2;
	double y2;
}gline;

typedef struct _signal
{
	double _angle1_radian;
	double _angle2_radian;
	double _angle1_degree;
	double _angle2_degree;
	double _role;
}Signal;

typedef struct _feature_point
{
	POINT _point;
	double _angle;
}featurepoint;

typedef struct _keyword
{
	int _index_role;
	int _index_theta_one;
	int _index_theta_two;
	std::vector<int> _frame_index;
}keyword;

#endif
