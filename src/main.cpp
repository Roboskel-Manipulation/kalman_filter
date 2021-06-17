#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <string>

#include <kalman_filter/KalmanFilterObj.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "kalmanFilter");
	ros::NodeHandle nh;

	std::string keypoint_topic = "/trajectory_points";
	int freq;
	bool online;

	nh.param("kalmanFilter/frequency", freq, 30);
	nh.param("kalmanFilter/online", online, false);

	ROS_INFO("keypoint topic: %s", keypoint_topic.c_str());

	KalmanFilterObj<geometry_msgs::PointStamped> kf_obj(freq, online);

	ros::Subscriber sub = nh.subscribe(keypoint_topic, 1, &KalmanFilterObj<geometry_msgs::PointStamped>::KalmanFilterCallback, &kf_obj);

	ros::spin();
}
