/**
   Node that reads an image using OpenCV and publishes it on topic
 */ 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdexcept>
#include <vector>
#include <string>


namespace nodedata
{

/// \brief The image filename to read
const std::string IMG_FILENAME = "test_img.JPG"; //"/home/david/ros_catkin_ws/my_catkin_ws/src/ocv_lane_tracker/datalane_find_image.png";
const std::string OPENCV_CAM_WINDOW = "OpenCV Camera Window";

}

int main(int argc, char **argv){

  //the name of the node this must be unique
  std::string node_name="OCVCamer";

  //initialize ROS with the node name
  ros::init(argc, argv, node_name);
	ros::NodeHandle node;

	image_transport::ImageTransport transport(node);
	image_transport::Publisher pub = transport.advertise("camera/image", 1);

	cv::namedWindow(nodedata::OPENCV_CAM_WINDOW);

	// Read & show the image we send
  cv::Mat image = cv::imread(nodedata::IMG_FILENAME); //, CV_LOAD_IMAGE_COLOR);

	if(!image.data){

		throw std::runtime_error("Could  not load image file: "+ nodedata::IMG_FILENAME);
	}
	else if((image.size().width == 0) || (image.size().height == 0)){
		
		throw std::runtime_error("Filename: " + nodedata::IMG_FILENAME + ", points to corrupted image");
	}

	cv::imshow(nodedata::OPENCV_CAM_WINDOW, image);
  cv::waitKey(30);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	ros::Rate loop_rate(5);
	
	while (node.ok()) {

       pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
  }
  
  return 0;
}
