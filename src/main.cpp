#include <ros/ros.h>
#include "laserline/laser_feature_ros.h"

using namespace std;

/*timeval endt1,start1;
double timeu1 = 0;

FILE * fp = fopen("timeuse.txt","w+");*/

int main(int argc,char** argv)
{
	ROS_DEBUG("Starting laserline node.");

	ros::init(argc, argv, "laserline");
	ros::NodeHandle nh;
	ros::NodeHandle nh_local("~");
	line_feature::LaserFeatureROS line_feature_ros(nh, nh_local);

/*	double frequency;
	nh_local.param<double>("frequency", frequency, 40);
	ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
	ros::Rate rate(frequency);

	while (ros::ok())
	{
		gettimeofday(&start1,0);
		line_feature_ros.startgame();
		gettimeofday(&endt1,0);
		timeu1 = 1000000*(endt1.tv_sec-start1.tv_sec)+endt1.tv_usec-start1.tv_usec;
		timeu1 /= 1000;
		
		fprintf(fp,"%lf\n",timeu1);		

		ros::spinOnce();
		rate.sleep();
	}*/
	return 0;
}
