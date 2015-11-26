/*
 * This source file is part of the Sensors and Sensing course at AASS.
 * If you use this material in your courses or research, please include 
 * a reference.
 * 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

//include headers from ROS
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//PCL
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
//EIGEN
#include <Eigen/Eigen>
//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//OPENCV Window names
#define RGB_WINDOW "RGB Image"
#define DEPTH_WINDOW "Depth Image"


int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;

cv::Mat src; cv::Mat dst;
char window_name[] = "Filter Demo 1";

/// Function headers
int display_caption( char* caption );
int display_dst( int delay );

struct windowDataPublisher
{
  windowDataPublisher(){};
  int windowSize;
  ros::Publisher variance_pub;
  ros::Publisher mean_pub;
  windowDataPublisher(ros::NodeHandle n_, int windowSize) : windowSize(windowSize) 
  {
    std::ostringstream os ;
    os << windowSize ;
    std::string r = os.str();
    variance_pub = n_.advertise<std_msgs::Float32>("/depth/variance_" + r, 1000);
    mean_pub = n_.advertise<std_msgs::Float32>("/depth/mean_"+ r, 1000);
  }
  
  void calcData(const cv::Mat& img)
  {
    
    int newSizeX = windowSize;
    int newSizeY = windowSize;

    
    cv::Mat window = img(cv::Range((img.rows-newSizeX)/2,(img.rows+newSizeX)/2),
			  cv::Range((img.cols-newSizeY)/2,(img.cols+newSizeY)/2));
    
    float sum = 0;
    int numPoints = 0;

    for(int i = 0; i < newSizeX; i++)
    {
      for(int j = 0; j < newSizeY; j++)
      {
	if(!std::isnan(window.at<float>(i,j)))
	{
	  sum += window.at<float>(i,j);
	  numPoints++;
	}
      }
    }
    float mean = sum / (numPoints);
    float stdevSum = 0;

    for(int i = 0; i < newSizeX; i++)
    {
      for(int j = 0; j < newSizeY; j++)
      {
	if(!std::isnan(window.at<float>(i,j)))
	{
	  stdevSum += (window.at<float>(i,j) - mean)*(window.at<float>(i,j) - mean);
    // 	      numPoints++;
	}

      }
    }
    float variance = (stdevSum / (float)numPoints);
    float stdev = sqrt(variance);
    // 	printf("Mean%f, std:%f, numpoints:%d\n",mean,stdev,numPoints);

    std_msgs::Float32 variance_msg;
    variance_msg.data = variance;
    std_msgs::Float32 mean_msg;
    mean_msg.data = mean;
    variance_pub.publish(variance_msg);
    mean_pub.publish(mean_msg);
  }
  
};


//Your Node Class
class AsusNode {

    private:
    // Our NodeHandle, points to home
    ros::NodeHandle nh_;
    //global node handle
    ros::NodeHandle n_;

    //Subscribers for topics
    ros::Subscriber points_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;
//     ros::Publisher variance_pub;
//     ros::Publisher mean_pub;

    //topics to subscribe to
    std::string subscribe_topic_point;
    std::string subscribe_topic_depth;
    std::string subscribe_topic_color;
    
    windowDataPublisher size40;
    windowDataPublisher size60;
    windowDataPublisher size80;

    public:
    AsusNode() {

	nh_ = ros::NodeHandle("~");
	n_ = ros::NodeHandle();
	
	size40 = windowDataPublisher(n_,40);
	size60 = windowDataPublisher(n_,60);
	size80 = windowDataPublisher(n_,80);
	
	//read in topic names from the parameter server
	nh_.param<std::string>("points_topic",subscribe_topic_point,"/camera/depth_registered/points");
	nh_.param<std::string>("depth_topic",subscribe_topic_depth,"/camera/depth_registered/image_raw");
	nh_.param<std::string>("rgb_topic",subscribe_topic_color,"/camera/rgb/image_raw");
	
	
// 	variance_pub = n_.advertise<std_msgs::Float32>("/depth/variance", 1000);
// 	mean_pub = n_.advertise<std_msgs::Float32>("/depth/mean", 1000);

	//subscribe to topics
	points_sub_ = n_.subscribe(subscribe_topic_point, 1, &AsusNode::points2Callback, this);
	depth_sub_ = n_.subscribe(subscribe_topic_depth, 1, &AsusNode::depthCallback, this);
	rgb_sub_ = n_.subscribe(subscribe_topic_color, 1, &AsusNode::rgbCallback, this);

	//create opencv windows
	cv::namedWindow(RGB_WINDOW);
	cv::namedWindow(DEPTH_WINDOW);
	cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    }

    // Callback for pointclouds
    void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
    {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*msg_in, cloud);

	/* do something pointy"*/
// 	ROS_INFO_STREAM("Got cloud with "<<cloud.size()<<" points");

    }

    //callback for rgb images
    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
	cv_bridge::CvImageConstPtr bridge;
	try
	{
	    bridge = cv_bridge::toCvCopy(msg, "bgr8");
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("Failed to transform rgb image.");
	    return;

	}
	

	/* do something colorful"*/
// 	cv::imwrite( "/home/anders/images/rgb.bmp", bridge->image);

	
	cv::imshow(RGB_WINDOW, bridge->image);
	cv::waitKey(1);
    }
    
    
    void saveDepthImages(cv::Mat img)
    {
      for(int i = 0; i < img.rows; i++)
      {
	for(int j = 0; j < img.cols; j++)
	{
	  img.at<float>(i,j) /= 7.0f;
	}
      }

      cv::Mat imgNoNaN;
      cv::Mat distImgFilter;
      cv::Mat distImgSave;
      
      
      img.convertTo(imgNoNaN, CV_32F);
      
      for(int i = 0; i < imgNoNaN.rows; i++)
      {
	for(int j = 0; j < imgNoNaN.cols; j++)
	{
	  if(std::isnan(imgNoNaN.at<float>(i,j)))
	  {
	      imgNoNaN.at<float>(i,j) = 0.0f;
	  }
	}
      }
      imgNoNaN.convertTo(distImgSave, CV_32S, 256.0);
      cv::imwrite( "/home/anders/images/depth.png", distImgSave);
      
      cv::medianBlur ( imgNoNaN, distImgFilter, 5 );
      distImgFilter.convertTo(distImgSave, CV_32S, 256.0);
      cv::imwrite( "/home/anders/images/depth_medianBlur.png", distImgSave);
      
      cv::GaussianBlur( imgNoNaN, distImgFilter, cv::Size( 5, 5 ), 0, 0 );
      distImgFilter.convertTo(distImgSave, CV_32S, 256.0);
      cv::imwrite( "/home/anders/images/depth_GaussianBlur.png", distImgSave);
      
      cv::bilateralFilter ( imgNoNaN, distImgFilter, 10, 5, 5 );
      distImgFilter.convertTo(distImgSave, CV_32S, 256.0);
      cv::imwrite( "/home/anders/images/depth_bilateralFilter.png", distImgSave);
      
      cv::imshow( "Filtered", distImgFilter);
//       display_dst( DELAY_BLUR );
    }
    

    //callback for RGB images
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
	cv_bridge::CvImageConstPtr bridge;

	try
	{
	    bridge = cv_bridge::toCvCopy(msg, "32FC1");
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("Failed to transform depth image.");
	    return;
	}
	
	cv::Mat img = bridge->image;
	
// 	saveDepthImages(img);
	
	cv::imshow(DEPTH_WINDOW, img);
	
	size40.calcData(img);
	size60.calcData(img);
	size80.calcData(img);
	
	/*
	int newSizeX = 40;
	int newSizeY = 40;

	
	cv::Mat window = img(cv::Range((img.rows-newSizeX)/2,(img.rows+newSizeX)/2),
			     cv::Range((img.cols-newSizeY)/2,(img.cols+newSizeY)/2));

	
	float sum = 0;
	int numPoints = 0;

	for(int i = 0; i < newSizeX; i++)
	{
	  for(int j = 0; j < newSizeY; j++)
	  {
	    if(!std::isnan(window.at<float>(i,j)))
	    {
	      sum += window.at<float>(i,j);
	      numPoints++;
	    }
	  }
	}
	float mean = sum / (numPoints);
	float stdevSum = 0;
	
	for(int i = 0; i < newSizeX; i++)
	{
	  for(int j = 0; j < newSizeY; j++)
	  {
	    if(!std::isnan(window.at<float>(i,j)))
	    {
	      stdevSum += (window.at<float>(i,j) - mean)*(window.at<float>(i,j) - mean);
// 	      numPoints++;
	    }

	  }
	}
	float variance = (stdevSum / (float)numPoints);
	float stdev = sqrt(variance);
// 	printf("Mean%f, std:%f, numpoints:%d\n",mean,stdev,numPoints);
	
	std_msgs::Float32 variance_msg;
	variance_msg.data = variance;
	std_msgs::Float32 mean_msg;
	mean_msg.data = mean;
	variance_pub.publish(variance_msg);
	mean_pub.publish(mean_msg);*/

// // 	  printf("img %d\n",bridge->image.data[i])int DELAY_CAPTION = 1500;

// 	  
// 	for(MatIterator_<cv::Mat> = bridge->image.begin();;)
// 	for(bridge->image.begin()bridge->image.begin()
// 	for(int& pixel : bridge->image)
// 	  pixel *= 4;
// roslaunch openni2_launch openni2.launch rgb_camera_info_url:="~/camera.yaml"
// rosrun  camera_calibration_parsers convert  /home/anders/ost.ini camera.yaml
	
// 	cv_bridge::CvImageConstPtr  img2;
// 	cv::Mat saving_image = cv::Mat::zeros(bridge->image.height*bridge->image.width);
// 	cvtColor( img2, bridge->image, CV_BGR2GRAY );
// 	cv::Mat saving_image;//  = bridge->image*4.0;
// 	bridge->image.convertTo(saving_image, CV_32S, 256.0);
	
	
// 	saving_image = saving_image * pow(2.0,24.0);
	
// 	for( int i = 0; i < 640*480; i++)
// 	  bridge->image.data[i] = bridge->image.data[i] << 24;
	 
// 	cv::imwrite( "/home/anders/images/depth2.png", saving_image);
// 	cv::imshow(DEPTH_WINDOW, window);
	
	
	
// 	bridge->image.convertTo(bridge->image, CV_32FC1, 255.0/0xffff);
	
	cv::waitKey(1);
    }

};




//  int testBlur(/*const cv::Mat& src*/)
//  {
//    cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );
// 
//    /// Load the source image
// //    src = cv::imread( "../images/lena.jpg", 1 );
// 
//    if( display_caption( "Original Image" ) != 0 ) { return 0; }
// 
//    dst = src.clone();
//    if( display_dst( DELAY_CAPTION ) != 0 ) { return 0; }
// 
//    /// Applying Homogeneous blur
//    if( display_caption( "Homogeneous Blur" ) != 0 ) { return 0; }
// 
//    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//        { cv::blur( src, dst, cv::Size( i, i ), cv::Point(-1,-1) );
//          if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }
// 
//     /// Applying Gaussian blur
//     if( display_caption( "Gaussian Blur" ) != 0 ) { return 0; }
// 
//     for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//         { cv::GaussianBlur( src, dst, cv::Size( i, i ), 0, 0 );
//           if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }
// 
//      /// Applying Median blur
// //      if( display_caption( "Median Blur" ) != 0 ) { return 0; }
// // 
//      for ( int i = 3; i <= 5; i = i + 2 )
//          { 
// 	   medianBlur ( src, dst, i );
//            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } 
// // 	    medianBlur ( src, dst, 3 );
// //            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } 
// // 	    medianBlur ( src, dst, 5 );
// //            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } 
// 	   
// 	}
//            
//            
// //                 for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
// //          { medianBlur ( src, dst, i );
// //            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }
// 
//      /// Applying Bilateral Filter
//      if( display_caption( "Bilateral Blur" ) != 0 ) { return 0; }
// 
//      for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//          { bilateralFilter ( src, dst, i, i*2, i/2 );
//            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }
// 
//      /// Wait until user press a key
//      display_caption( "End: Press a key!" );
// 
// //      cv::waitKey(0);
//      return 0;
//  }

 int display_caption( char* caption )
 {
   dst = cv::Mat::zeros( src.size(), src.type() );
   cv::putText( dst, caption,
            cv::Point( src.cols/4, src.rows/2),
            CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255) );

   cv::imshow( window_name, dst );
//    int c = cv::waitKey( DELAY_CAPTION );
//    if( c >= 0 ) { return -1; }
   return 0;
  }

  int display_dst( int delay )
  {
    cv::imshow( window_name, dst );
    int c = cv::waitKey ( delay );
    return 0;
//     if( c >= 0 ) { return -1; }
//     return 0;
  }


//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "asus_node");

    std::cerr<<"creating node\n";
    AsusNode nd;
    std::cerr<<"node done\n";
    
    int i = 1;
    ros::Rate loop_rate(10);
    while (ros::ok()) {

//       int c = cv::waitKey ( 10 );
//       if( c == 'd' ) 
//       { 
//       if(i%50 == 0) 
//       { 
// 	testBlur();
// 	
//       }
//       i++;
      
      ros::spinOnce();
      loop_rate.sleep();
    }				
    
//     ros::spin();

    return 0;
}




