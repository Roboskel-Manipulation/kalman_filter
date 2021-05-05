## kalman_filter_pkg

![Build Status](https://upload.wikimedia.org/wikipedia/commons/thumb/3/32/OpenCV_Logo_with_text_svg_version.svg/180px-OpenCV_Logo_with_text_svg_version.svg.png)
<img src="http://wiki.ros.org/melodic?action=AttachFile&do=get&target=melodic.jpg" width="220">
<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcQhykQfMn6vBr8q24DRapZ_PqBerW491szxvg&usqp=CAU" width="220">

### Description
A ROS package with an implemented kalman filter, using the OpenCV [KalmanFilter class](https://docs.opencv.org/master/dd/d6a/classcv_1_1KalmanFilter.html#a077d73eb075b00779dc009a9057c27c3). \
The main node accepts `geometry_msgs/PointStamped` points and it applies a Kalman filter. The input topic is defined in the config file `params.yaml`. The filtered points are published to the topic `/kalman_points`.

### Parameters
* `keypoint_topic`: Input topic
* `online`: True if the prediction is based on the timestamp of the messages (default: False)
* `frequency`: Used in the prediction only if `online` is set to False. (default: 30 => visual sensor's frequency)
* `Rx, Ry, Rz`: Noise covariance matrix
* `Q`: Process covariance matrix

### Run
- Run `roslaunch kalman_filter kalman.launch`
