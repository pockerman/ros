#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <string>


namespace nodedata
{

static const std::string OPENCV_WINDOW = "Lane Detector Window";
static const std::string SUSBSCRIBE_TOPIC_NAME = "/camera/image_raw";
static const std::string ADVERTISED_TOPIC_NAME = "/image_converter/output_video";
static const int EDGE_THESH = 1;
static int LOW_THRESHOLD;
static const int MAX_LOW_THRESHOLD = 100;
static const int RATIO = 3;
static const int KERNEL_SIZE = 3;
  

class LaneDetector
{

public:

        /// \brief Constructor
        LaneDetector(ros::NodeHandle& node);

        /// \brief Destructor
        ~LaneDetector();

        /// \brief This is a callback for input_image_topic.
        ///  What it basically does is that it converts the sensor_msgs/Image data
        /// into the cv::Mat OpenCV data type. The cv_bridge::CvImagePtr cv_ptr buffer is
        /// allocated for storing the OpenCV image after performing the ROS-OpenCV conversion
        void image_cb(const sensor_msgs::ImageConstPtr& msg);

				/// \brief Tracks lane from the given image
				void track_lane();
private:


		/// \brief Helper function to detect edges into an image
		/// By default it uses the Canny edge detection algorithm 		
		void canny_edge_detection_(cv_bridge::CvImagePtr& ptr);

		/// \brief Apply Gaussian blurring to the input image
		/// By default it uses a 5x5 kernel with 0 standard deviation
		void apply_gaussian_blurring_(cv_bridge::CvImagePtr& ptr);


		/// \brief Identifies the region of ineterst (ROI)
		/// it specifies the closed region of the FoV
		void identify_roi_(cv_bridge::CvImagePtr& ptr);

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

		/// \brief Matrix that holds the detected edges
		cv::Mat detected_edges_;
};

LaneDetector::LaneDetector(ros::NodeHandle& node)
    :
   it_(node),
   image_sub_(),
   image_pub_()
{
    // Subscrive to input video feed and publish output video feed
     image_sub_ = it_.subscribe(SUSBSCRIBE_TOPIC_NAME, 1, &LaneDetector::image_cb, this);
     image_pub_ = it_.advertise(ADVERTISED_TOPIC_NAME, 1);
     cv::namedWindow(OPENCV_WINDOW);
}

LaneDetector::~LaneDetector()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void
LaneDetector::image_cb(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		//convert image to greyscale 
		cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2GRAY);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}


void 
LaneDetector::apply_gaussian_blurring_(cv_bridge::CvImagePtr& cv_ptr){

	cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(5, 5), 0);
}

void 
LaneDetector::canny_edge_detection_(cv_bridge::CvImagePtr& cv_ptr){

	cv::Canny( cv_ptr->image, detected_edges_, LOW_THRESHOLD, LOW_THRESHOLD*RATIO, KERNEL_SIZE );
}

void 
LaneDetector::identify_roi_(cv_bridge::CvImagePtr& ptr){

	
	auto img_h = ptr->image.size().height;

	// the following points specify the closed ROI
	// of the FoV we are interested in
	// vertex 1: 200, img_h
	// vertex 2: 1100, img_h
	// vertex 3: 550, 250

	// black mask with the same dimensions and type
  // as the incoming image
	cv::Mat mask (ptr->image.size(), ptr->image.type(), cv::Scalar::all(0)); 

	// fill the mask with the ROI

	cv::Point v1;
	cv::Point v2;
	cv::Point v3;

	const cv::Point* triangle[] = { &v1, &v2, &v3 };
  int npt[] = { 3 };
	int lineType = 8;

	// see http://www.swarthmore.edu/NatSci/mzucker1/opencv-2.4.10-docs/doc/tutorials/core/basic_geometric_drawing/basic_geometric_drawing.html
	cv::fillPoly(mask, triangle, npt, 1, cv::Scalar::all(255), lineType );

	// return the mask we created

	// do a bitwise_and operation and store the result in the original image
	cv::bitwise_and(ptr->image, mask, ptr->image);
}

void
LaneDetector::track_lane(){

//apply_gaussian_blurring_(cv_bridge::CvImagePtr& cv_ptr);
//canny_edge_detection_(cv_bridge::CvImagePtr& cv_ptr);
}
}




int main(int argc, char **argv){

  //the name of the node this must be unique
  std::string node_name="OCVLaneTracker";

  //initialize ROS with the node name
  ros::init(argc, argv, node_name);

  ros::NodeHandle node;
  nodedata::LaneDetector detector(node);
  ros::spin();

  return 0;
}
