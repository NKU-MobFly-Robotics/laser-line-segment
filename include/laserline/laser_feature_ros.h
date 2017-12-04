/*
mobile robot pose from laser
*/

#ifndef _LASER_FEATURE_ROS_H_
#define _LASER_FEATURE_ROS_H_

#include <vector>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "laserline/line_feature.h"

namespace line_feature
{

class LaserFeatureROS
{
    public:
		LaserFeatureROS(ros::NodeHandle&, ros::NodeHandle&);
		//
		~LaserFeatureROS();
		//
		void startgame();
	private:
		//memeber function
		//发布直线分割消息
		void publishMarkerMsg(const std::vector<gline> &,visualization_msgs::Marker &marker_msg);
		//开始函数，包括读取文件（先验地图等信息）
		//load params
		void load_params();
		//角度参量
		void compute_bearing(const sensor_msgs::LaserScan::ConstPtr&);
		//激光线程函数（ros节点回调函数），采集激光并进行处理，处理频率以采集频率为准
		void scanValues(const sensor_msgs::LaserScan::ConstPtr&);

     private:
		//参数信息laser
		bool com_bearing_flag;
		bool show_lines_;
		double m_startAng;
		double m_AngInc;
		LineFeature line_feature_;

		std::string frame_id_;
  		std::string scan_topic_;

		std::vector<gline> m_gline;
		std::vector<line> m_line;
		//ROS
		ros::NodeHandle nh_;
  		ros::NodeHandle nh_local_;
  		ros::Subscriber scan_subscriber_;
  		ros::Publisher line_publisher_;
  		ros::Publisher marker_publisher_;
};

}
#endif

