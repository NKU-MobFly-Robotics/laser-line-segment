/*
written by nanaki ghm
laser_line_extraction

*/

#ifndef _LINE_FEATURE_H_
#define _LINE_FEATURE_H_

#include <laserline/fstruct.h>
#include <iostream>
#include <stdio.h>
#include <math.h>

//这些模块可以用自己的math.h头文件进行包含
////宏定义
#define PI 3.1415926535898
#define t(a) a*PI/180 //角度弧度转换
//识别模块定义
#define threshold_a 6     //聚类倍数
#define distance(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))
#define max(a,b) a>b?a:b

namespace line_feature
{

class LineFeature
{
	public:
		LineFeature();
		//
		~LineFeature();
		//子函数声明
		//设置bearing，一次设置就行
		void setCosSinData(const std::vector<double>&, const std::vector<double>&,const std::vector<double>&, const std::vector<unsigned int>&);
		//设置range，每次都需要传递range消息，在主入口函数的回调函数进行
		void setRangeData(const std::vector<double>&); 
		//返回直线分割结果
		void extractLines(std::vector<line>&,std::vector<gline>&);
		//设置参数
		void set_angle_increment(double);
		void set_angle_start(double);
		void set_least_threshold(double);
		void set_min_line_length(double);
		void set_predict_distance(double);
		void set_min_line_points(unsigned int);
		void set_seed_line_points(unsigned int);
	private:
		//检测种子点，障碍物聚类的子函数
		//bool detectseed(PoinT );
		//障碍物检测，区域生长
		//int regiongrow(int);
		//通过激光数据的首末索引值进行直线方程的求解
		least leastsquare(int,int,int);
		//检测种子直线
		bool detectline(const int,const int);
		//通过种子直线，复原出整条直线，并进行最后的求定
		int detectfulline(const int);
		//整理整条直线
		void cleanline();
		//删除小于长度阈值的线段
		bool delete_short_line(const int,const int);
		//
		void generate(std::vector<gline>& temp_line2);
	private:
		CSdata cs_data_;
		Rangedata range_data_;
		Params params_;
		std::vector<unsigned int> point_num_;
		//线段结构体信息
		std::vector<line> m_line;
		//直线拟合中间传递变量，已设为全局变量
		least m_least;
        //拟合中间变量
		double mid1;
		double mid2;
		double mid3;
		double mid4;
		double mid5;
};

}//namespace line_feature
#endif

