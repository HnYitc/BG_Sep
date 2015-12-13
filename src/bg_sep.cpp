#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam_node/image_fix", 1, 
      &ImageConverter::imageCb, this);
//    image_pub_ = it_.advertise("/image_bgSep", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    int key = cv::waitKey(1);

    static Mat bgImg = Mat::zeros(cv::Size(cv_ptr->image.cols,cv_ptr->image.rows), CV_8U);
    static Mat clcImg = Mat::zeros(cv::Size(cv_ptr->image.cols,cv_ptr->image.rows), CV_8U);

    double ratio = 0;

    if(key == 's'){

      for (int y = 0; y < cv_ptr->image.rows; y++) {
        for (int x = 0; x < cv_ptr->image.cols; x++) {

          bgImg.at<uchar>(y,x) = cv_ptr->image.at<Vec3b>(y, x)[2];

        }
      }

      cout << "bgImage captured!\n" << endl;

    }

    for (int y = 0; y < cv_ptr->image.rows; y++) {
      for (int x = 0; x < cv_ptr->image.cols; x++) {

        clcImg.at<uchar>(y,x) = abs(cv_ptr->image.at<Vec3b>(y, x)[2] - bgImg.at<uchar>(y, x));

      }
    }

    static unsigned int ts = 0;
    if(key == 'u') ts = ts + 1;
    if(key == 'd') ts = ts - 1;

    threshold( clcImg, clcImg, ts, 255, THRESH_BINARY);

    double sum = 0;
    int count = 0;
    for (int y = 0; y < cv_ptr->image.rows; y++) {
      for (int x = 0; x < cv_ptr->image.cols; x++) {

        int v = clcImg.at<uchar>(y,x);
        sum += (double)v;
        count++;

      }
    }
    ratio = sum/(double)count;

    cout << "ratio:" << ratio << "\n" << endl;

    

    cv::imshow("bgImg", bgImg);
    cv::imshow("clcImg", clcImg);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "BackGround_Separater");
  ImageConverter ic;
  ros::spin();
  return 0;
}

