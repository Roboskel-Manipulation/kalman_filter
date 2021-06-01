#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "std_msgs/Float64MultiArray.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>

#include <kalman_filter/KalmanFilterObj.h>

/****
Kalman Filter Variables

state: estimated state variables, contains all xyz coordinates for each keypoint
measurement: xyz coordinates of keypοints
H: measurement matrix
F: transition matrix
Q: process noise covariance
R: measurement noise covariance
P: estimation error covariance

****/

unsigned int type = CV_64FC1;
ros::Publisher vel;

template <class T> 
KalmanFilterObj<T>::KalmanFilterObj(float _freq, bool _online){

	int i;
	ros::NodeHandle nh;
	if(!_online){
		this->dT = 1/_freq;
		printf("Message intervals: %lf\n",this->dT);
	}
	this->online = _online;
	nh.param("kalmanFilter/Rx", this->Rx, 0.0);
	nh.param("kalmanFilter/Ry", this->Ry, 0.0);
	nh.param("kalmanFilter/Rz", this->Rz, 0.0);
	nh.param("kalmanFilter/Q", this->Q, 0.0);

	pub = nh.advertise<geometry_msgs::PointStamped>("/kalman_points", 10000);
	debug_pub = nh.advertise<geometry_msgs::PointStamped>("/debug_kalman_points", 10000);
	vel = nh.advertise<std_msgs::Float64MultiArray>("/velocity", 10000);
	Init();
	ROS_INFO("Ready to accept points");

}

/* Initialize parameters for Kalman Filter */
template <class T> 
void KalmanFilterObj<T>::Init(){
	int i;

	this->measLen = 3; /* xyz */

	this->stateLen = measLen;

	kf = cv::KalmanFilter(stateLen, measLen, measLen, type);

	state = cv::Mat(stateLen, 1, type);
	measurement = cv::Mat(measLen, 1, type);
	velocity = cv::Mat(measLen, 1, type);

	timer = cv::Mat(measLen, 1, type);
	timer.setTo(0);

	x_t1 = cv::Mat(measLen, 1, type);
	x_t2 = cv::Mat(measLen, 1, type);

	cv::setIdentity(kf.transitionMatrix);

	kf.measurementNoiseCov.at<double>(0,0) = this->Rx;
	kf.measurementNoiseCov.at<double>(1,1) = this->Ry;
	kf.measurementNoiseCov.at<double>(2,2) = this->Rz;

	cv::setIdentity(kf.processNoiseCov, cv::Scalar(this->Q));
	kf.measurementMatrix = cv::Mat::zeros(measLen, stateLen, type);
	cv::setIdentity(kf.measurementMatrix);
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
	cv::setIdentity(kf.controlMatrix);
}

double currentTime = 0.0, curdT = 0.0;
cv::Mat temp_vel; /* here we store previous velocity */
cv::Mat predicted, corrected; /* kalman predicted/corrected keypoints */
geometry_msgs::PointStamped prev_keypoints; /* used to spot outliers */
cv::Mat was_outlier; /* true if previous keypoint measurement was an outlier */

template <class T> 
void KalmanFilterObj<T>::KalmanFilterCallback(const T msg){

	currentTime = msg.header.stamp.toSec(); /* time in seconds */

	curdT = currentTime - this->prevTime;

	if(curdT == 0.0 && this->online){
		ROS_INFO("***Zero Time Difference***");
		pub.publish(msg);
		return;
	}

	this->prevTime = currentTime;
	if(this->online){
		std::cout << "[Current Difference: " << std::setprecision(25) << curdT << "]" << std::endl;
		this->dT = curdT;
	}

	if(this->first_call){
		this->first_call = false;
		this->second_call = true;

		geometry_msgs::PointStamped prev_keypoints;

		ROS_INFO("First Callback!");
		
		x_t2.at<double>(0) = msg.point.x;
		x_t2.at<double>(1) = msg.point.y;
		x_t2.at<double>(2) = msg.point.z;
		
		/* publish the initial measurements unchanged*/
		pub.publish(msg);
		return;
	}

	if(this->second_call){
		this->second_call = false;
		ROS_INFO("Second Callback!");

		/* initialize state and velocity */
		state.at<double>(0) = msg.point.x;
		state.at<double>(1) = msg.point.y;
		state.at<double>(2) = msg.point.z;

		prev_keypoints.point.x = msg.point.x;
		prev_keypoints.point.y = msg.point.y;
		prev_keypoints.point.z = msg.point.z;

		state.copyTo(x_t1);

		cv::subtract(x_t1, x_t2, velocity);
		velocity = velocity * (1/this->dT); /* velocity = (x_t1 - x_t2)/dT */
		velocity.copyTo(temp_vel);

		pub.publish(msg);

		kf.statePost = state;
		return;
	}

	measurement.at<double>(0) = msg.point.x;
	measurement.at<double>(1) = msg.point.y;
	measurement.at<double>(2) = msg.point.z;

	/* fix controlMatrix with current dT */
	timer = timer + this->dT;
	kf.controlMatrix.at<double>(0, 0) = timer.at<double>(0);
	kf.controlMatrix.at<double>(1, 1) = timer.at<double>(1);
	kf.controlMatrix.at<double>(2, 2) = timer.at<double>(2);

	/* Increase the measurement noise for the third point because Kalman gain is
	   initialized to 1 and the third filtered point is the almost the same with
	   the measurement. For the rest points, use the user-defined measurement noise */
	if (this->third_call){
		kf.measurementNoiseCov.at<double>(0, 0) = 1;
		kf.measurementNoiseCov.at<double>(1, 1) = 1;
		kf.measurementNoiseCov.at<double>(2, 2) = 1;
	}
	else{
		kf.measurementNoiseCov.at<double>(0,0) = this->Rx;
		kf.measurementNoiseCov.at<double>(1,1) = this->Ry;
		kf.measurementNoiseCov.at<double>(2,2) = this->Rz;
	}

	/* predict state */
	predicted = kf.predict(velocity);

	/* update state */
	corrected = kf.correct(measurement);

	/* create new point msg with corrected state */
	this->corrected_msg.header = msg.header;
	this->corrected_msg.point.x = corrected.at<double>(0);
	this->corrected_msg.point.y = corrected.at<double>(1);
	this->corrected_msg.point.z = corrected.at<double>(2);
	this->debug_msg.header  = msg.header;
	this->debug_msg.point.x = predicted.at<double>(0);
	this->debug_msg.point.y = predicted.at<double>(1);
	this->debug_msg.point.z = predicted.at<double>(2);

	pub.publish(this->corrected_msg);
	debug_pub.publish(this->debug_msg);

	/* calculate new velocity */
	x_t1.copyTo(x_t2); // x_t2 <- x_t1
	// measurement.copyTo(x_t1); // x_t1 <- measurement

	x_t1.at<double>(0) = corrected.at<double>(0);
	x_t1.at<double>(1) = corrected.at<double>(1);
	x_t1.at<double>(2) = corrected.at<double>(2);

	cv::subtract(x_t1,x_t2,velocity);
	cv::divide(velocity, timer, velocity);

	timer.at<double>(0) = 0;
	timer.at<double>(1) = 0;
	timer.at<double>(2) = 0;

	velocity.copyTo(temp_vel);

	/* update previous keypoint values */
	prev_keypoints.point.x = x_t1.at<double>(0);
	prev_keypoints.point.y = x_t1.at<double>(1);
	prev_keypoints.point.z = x_t1.at<double>(2);
}

template class KalmanFilterObj<geometry_msgs::PointStamped>;
