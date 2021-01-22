#include <ros/ros.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

#include <string>

#include <kalman_filter/KalmanFilterObj.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "kalmanFilter");
	ros::NodeHandle nh;

	std::string keypoint_topic;
	int freq;
	bool online;

	nh.param("kalmanFilter/keypoint_topic", keypoint_topic, std::string("/raw_points_online"));
	nh.param("kalmanFilter/frequency", freq, 30);
	nh.param("kalmanFilter/online", online, false);

	ROS_INFO("keypoint topic: %s", keypoint_topic.c_str());

	KalmanFilterObj<keypoint_3d_matching_msgs::Keypoint3d_list> kf_obj(freq, online);

	ros::Subscriber sub = nh.subscribe(keypoint_topic, 1, &KalmanFilterObj<keypoint_3d_matching_msgs::Keypoint3d_list>::KalmanFilterCallback, &kf_obj);
	
	ros::spin();
}
