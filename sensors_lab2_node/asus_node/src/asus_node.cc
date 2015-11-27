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
// #include <array>
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

#define SPICE 0
#define SHOW_IMG 1
#define SAVE_IMG 1


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
  windowDataPublisher(ros::NodeHandle n_, int windowSize, std::string name) : windowSize(windowSize) 
  {
//     std::ostringstream os ;
//     os << windowSize ;
//     std::string r = os.str();
    variance_pub = n_.advertise<std_msgs::Float32>("/depth/" + name + "/variance", 1000);
    mean_pub = n_.advertise<std_msgs::Float32>("/depth/"+ name + "/mean", 1000);
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

struct CircularList
  {
    int curImgIndex;
    std::vector<cv::Mat> images;
    std::vector<float> values;
    CircularList() : images(10), curImgIndex(0), values(10,NAN) {}

    void addImg(const cv::Mat& img)
    {
      images[curImgIndex%images.size()] = img;
      curImgIndex++;
    }
    
  cv::Mat movingMeanFilter()
  {
    
    cv::Mat meanImg = cv::Mat(images[0].size(),images[0].type());
    if(curImgIndex > 10)
    {

// 	    cv::accumulate(meanImg,imagesCircularList[i]);
      for(int i = 0; i < images[0].rows; i++)
      {
	for(int j = 0; j < images[0].cols; j++)
	{
	  float points = 0;
	  float value = 0;
// 	      meanImg.at<float>(i,j) = 0;
	  for(int k = 0; k < images.size(); k++)
	  {
	    if(!std::isnan(images[k].at<float>(i,j)))
	    {
	      value += images[k].at<float>(i,j);
	      points++;
	    }
	  }
	  meanImg.at<float>(i,j) = value/points;
// 	      printf("Failed to %f\n",meanImg.at<float>(i,j));
	}
      }
// 	   ROS_ERROR("Failed to transform depth image.");
      
// 	cv::imshow("DEPTH_WINDOW_mean", meanImg/7.0);
    }
    return meanImg;
  }
  
  cv::Mat movingMedianFilter()
  {
    cv::Mat medianImg = cv::Mat(images[0].size(),images[0].type());
    if(curImgIndex > 10)
    {
      for(int i = 0; i < images[0].rows; i++)
      {
	for(int j = 0; j < images[0].cols; j++)
	{
	  float points = 0;
	  values.clear();
	  for(int k = 0; k < images.size(); k++)
	  {
	    float value = images[k].at<float>(i,j);
	    if(!std::isnan(value))
	    {
	      values.push_back(value);
// 	      for(int m = 0; m <= values.size(); m++)
// 	      {
// 		if(value < values[m])
// 		{
// 		  values.insert(values.begin()+m,value);
// 		  break;
// 		}
// 		else if(m == values.size())
// 		{
// 		  values.push_back(value);
// 		  break;
// 		}
// 	      }
	    }
	  }
	  std::sort(values.begin(),values.end());
	  if(values.size() == 0)
	    medianImg.at<float>(i,j) = NAN;
	  else
	    medianImg.at<float>(i,j) = values[values.size()/2];
	}
      }

// 	cv::imshow("medianImg", medianImg/7.0);
    }
    return medianImg;
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

    //topics to subscribe to
    std::string subscribe_topic_point;
    std::string subscribe_topic_depth;
    std::string subscribe_topic_color;
    // 	img2 = movingMean.movingMeanFilter(img.clone());
// 	cv::imshow("MovingMeanFilter", img2/7.0f);
    windowDataPublisher size40_org;
    windowDataPublisher size60_org;
    windowDataPublisher size80_org;
    
    windowDataPublisher size40_mean;
    windowDataPublisher size60_mean;
    windowDataPublisher size80_mean;
    
    windowDataPublisher size40_gauss;
    windowDataPublisher size60_gauss;
    windowDataPublisher size80_gauss;
    
    windowDataPublisher size40_bilat;
    windowDataPublisher size60_bilat;
    windowDataPublisher size80_bilat;
   
    windowDataPublisher size40_mean10;
    windowDataPublisher size60_mean10;
    windowDataPublisher size80_mean10;
    
    windowDataPublisher	size40_median;
    windowDataPublisher size60_median;
    windowDataPublisher size80_median;
    
    CircularList movingMean;
//     CircularList movingMean_60;
//     CircularList movingMean_80;
    


    public:
    AsusNode() {

	nh_ = ros::NodeHandle("~");
	n_ = ros::NodeHandle();
	
	size40_org = windowDataPublisher(n_,40,"org_40");
	size60_org = windowDataPublisher(n_,60,"org_60");
	size80_org = windowDataPublisher(n_,80,"org_80");
	
	size40_mean = windowDataPublisher(n_,40,"mean_40");
	size60_mean = windowDataPublisher(n_,60,"mean_60");
	size80_mean = windowDataPublisher(n_,80,"mean_80");
	
	size40_gauss = windowDataPublisher(n_,40,"gauss_40");
	size60_gauss = windowDataPublisher(n_,60,"gauss_60");
	size80_gauss = windowDataPublisher(n_,80,"gauss_80");
	
	size40_bilat = windowDataPublisher(n_,40,"bilat_40");
	size60_bilat = windowDataPublisher(n_,60,"bilat_60");
	size80_bilat = windowDataPublisher(n_,80,"bilat_80");
	
	size40_mean10 = windowDataPublisher(n_,40,"mean10_40");
	size60_mean10 = windowDataPublisher(n_,60,"mean10_60");
	size80_mean10 = windowDataPublisher(n_,80,"mean10_80");
	
	size40_median = windowDataPublisher(n_,40,"median_40");
	size60_median = windowDataPublisher(n_,60,"median_60");
	size80_median = windowDataPublisher(n_,80,"median_80");
	
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
// 	cv::namedWindow(RGB_WINDOW);
// 	cv::namedWindow(DEPTH_WINDOW);
// 	cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    }
    
    void dispDepthImg(std::string name, cv::Mat img)
    {
#if SHOW_IMG
      cv::imshow( name, img/7.0f );
#endif
#if SAVE_IMG
      
#if SPICE
      name += "_spice";
#endif
      (img.clone()).convertTo(img, CV_32S, 256.0/7.0);
      cv::imwrite( "/home/anders/images/depth_"+ name +".png", img);
#endif
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
	{// 	size40_mean.calcData(img3);
// 	size60_mean.calcData(img3);
// 	size80_mean.calcData(img3);
	    ROS_ERROR("Failed to transform rgb image.");
	    return;

	}
	

	/* do something colorful"*/
	cv::imwrite( "/home/anders/images/rgb.bmp", bridge->image);

	
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
	cv::Mat img2;
	cv::Mat img3;
	
#if SPICE
	for(int i = 0; i < 1000; i++)
	{
	  int grainX = rand() % img.rows;
	  int grainY = rand() % img.cols;
	  int spice = rand() % 2;
	  
	  if(!std::isnan(img.at<float>(grainX,grainY)))
	    img.at<float>(grainX,grainY) = spice*7.0;
	}
#endif
	

// 	
// 	img2 = movingMean.movingMeanFilter(img);
// 	cv::imshow("MovingMeanFilter", img2/7.0f);
	
	dispDepthImg("Original", img);
// 	cv::imshow(/7.0f);
// 
// 
// 	cv::Mat3 imgOrg
// 	(img.clone()).convertTo(imgOrg, CV_32S, 256.0);
// 	cv::imwrite( "/home/anders/images/depth_"+ name +".png", imgOrg);
	
	
//       img.convertTo(imgNoNaN, CV_32F);
//       
//       for(int i = 0; i < imgNoNaN.rows; i++)
//       {
// 	for(int j = 0; j < imgNoNaN.cols; j++)
// 	{
// 	  if(std::isnan(imgNoNaN.at<float>(i,j)))
// 	  {
// 	      imgNoNaN.at<float>(i,j) = 0.0f;
// 	  }
// 	}
//       }
// 	img.convertTo(img, CV_32FC3);
// 	saveDepthImages(img);
	size40_org.calcData(img);
	size60_org.calcData(img);
	size80_org.calcData(img);
	
	movingMean.addImg(img.clone());
	img2 = movingMean.movingMeanFilter();
	dispDepthImg("MovingMeanFilter", img2);
	size40_mean10.calcData(img2);
	size60_mean10.calcData(img2);
	size80_mean10.calcData(img2);
// 	cv::imshow("MovingMeanFilter", img2/7.0);
	img2 = movingMean.movingMedianFilter();
	dispDepthImg("MovingMedianFilter", img2);
// 	cv::imshow("MovingMedianFilter", img2/7.0);
	size40_median.calcData(img2);
	size60_median.calcData(img2);
	size80_median.calcData(img2);


	img2 = img;
	cv::medianBlur ( img2.clone(), img3, 5 );
	dispDepthImg("medianBlur", img3);
// 	cv::imshow("medianBlur", img3/7.0);
	size40_mean.calcData(img3);
	size60_mean.calcData(img3);
	size80_mean.calcData(img3);

	cv::GaussianBlur( img2.clone(), img3, cv::Size( 5, 5 ), 0, 0 );
	dispDepthImg("GaussianBlur", img3);
// 	cv::imshow("GaussianBlur", img3/7.0);
	size40_gauss.calcData(img3);
	size60_gauss.calcData(img3);
	size80_gauss.calcData(img3);
	
	cv::Mat imgNoNaN = img.clone();
	for(int i = 0; i < imgNoNaN.rows; i++)
	{
	  for(int j = 0; j < imgNoNaN.cols; j++)
	  {
	    if(std::isnan(imgNoNaN.at<float>(i,j)))
	    {
	      if(i != 0)
	      {
		imgNoNaN.at<float>(i,j) = imgNoNaN.at<float>(i-1,j);
	      }
	      else if(j != 0)
		imgNoNaN.at<float>(i,j) = imgNoNaN.at<float>(i,j-1);
	      else
		imgNoNaN.at<float>(i,j) = 0;
	    }
	  }
	}
	img2 = imgNoNaN;

	cv::bilateralFilter ( img2, img3, 6, 3, 3 );
	
	for(int i = 0; i < img3.rows; i++)
	{
	  for(int j = 0; j < img3.cols; j++)
	  {
	    if(std::isnan(img.at<float>(i,j)))
	    {
		img3.at<float>(i,j) = NAN;
	    }
	  }
	}
// 	img2 = imgNoNaN;
	dispDepthImg("bilateral", img3);
// 	cv::imshow("bilateral", img3/7.0);
	size40_bilat.calcData(img3);
	size60_bilat.calcData(img3);
	size80_bilat.calcData(img3);
	

	

// 	cv::waitKey(1);
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

//  int display_caption( char* caption )
//  {
//    dst = cv::Mat::zeros( src.size(), src.type() );
//    cv::putText( dst, caption,
//             cv::Point( src.cols/4, src.rows/2),
//             CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255) );
// 
//    cv::imshow( window_name, dst );
// //    int c = cv::waitKey( DELAY_CAPTION );
// //    if( c >= 0 ) { return -1; }
//    return 0;
//   }
// 
//   int display_dst( int delay )
//   {
//     cv::imshow( window_name, dst );
//     int c = cv::waitKey ( delay );
//     return 0;
// //     if( c >= 0 ) { return -1; }
// //     return 0;
//   }


//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "asus_node");

    std::cerr<<"creating node\n";
    AsusNode nd;
    std::cerr<<"node done\n";
    
//     int i = 1;
//     ros::Rate loop_rate(30);
/*    while (ros::ok()) {

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
    }	*/			
    
    ros::spin();

    return 0;
}




