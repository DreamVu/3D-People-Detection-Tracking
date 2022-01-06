#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>
#include <sys/time.h>

#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "PAL.h"

#include <std_msgs/Int32.h>

#include <sys/time.h>

using namespace std;
using namespace cv;
using namespace PAL;

static const float Pi = 3.1415926535898f;

/*
Specify the absolute file path from which the settings are to be read.

If the specified file can't be opened, default properties from the API are used.
See PAL Documentation for more information.
*/
#define PROPERTIES_FILE_PATH "../catkin_ws/src/dreamvu_pal_tracking/src/SavedPalProperties.txt"
                              

static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub1;


void publishimage(cv::Mat imgmat, image_transport::Publisher &pub, string encoding, timeval timestamp)
{
	int type;
	if (encoding == "mono8")
		type = CV_8UC1;
	else if (encoding == "mono16")
		type = CV_16SC1;
	else
		type = CV_8UC3;

    std_msgs::Header header;
    header.stamp.sec = timestamp.tv_sec;
    header.stamp.nsec = timestamp.tv_usec*1000;
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, encoding, imgmat).toImageMsg();

	pub.publish(imgmsg);
}

void set_id(const std_msgs::Int32& msg)
{
    int id = msg.data;
    PAL::SetTrackID(id);

    //std::cout<<"Set id to "<<id<<std::endl;

}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "following_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	//Creating all the publishers
	leftpub1 = it.advertise("/dreamvu/pal/tracking/get/left", 1);		
    ros::Subscriber id_sub = nh.subscribe("/dreamvu/pal/set/id", 1, set_id);
    
	int width, height;
    PAL::Mode mode = PAL::Mode::FOLLOWING;
    if(PAL::Init(width, height,-1, &mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }


	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(PROPERTIES_FILE_PATH, &g_CameraProperties);

	
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL settings from properties file at default location.\n\n"
				 "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in tracking_node.cpp and run catkin_make to build the package again.");
		ROS_INFO("Setting default properties to PAL.");

	}


	ros::Rate loop_rate(30);
	g_bRosOK = ros::ok();
    

    for(int i=0; i<10; i++)
    {
            PAL::Data::TrackingResults discard;
            discard = GrabTrackingData();
    }
    
	while (g_bRosOK)
	{

		//Getting no of subscribers for each publisher
		int left1Subnumber = leftpub1.getNumSubscribers();
		int subnumber = left1Subnumber;
		
        PAL::Data::TrackingResults data1;
        
		if (subnumber > 0)
		{
			
        	data1 = GrabTrackingData();

        #if 0		
	        for (int i = 0; i < data1.trackingData[PAL::States::OK].size(); i++)
	            cout << " in OK "<< data1.trackingData[PAL::States::OK][i].t_track_id << endl;

	        for (int i = 0; i < data1.trackingData[PAL::States::SEARCHING].size(); i++)
	            cout << " in SEARCHING "<< data1.trackingData[PAL::States::SEARCHING][i].t_track_id << endl;

	        for (int i = 0; i < data1.trackingData[PAL::States::TERMINATED].size(); i++)
	            cout << " in TERMINATED" << data1.trackingData[PAL::States::TERMINATED][i].t_track_id << endl;          
		#endif
		
			ros::WallTime t2 = ros::WallTime::now();			

		}

		if (left1Subnumber > 0)
		{
            publishimage(data1.left, leftpub1, "bgr8", data1.timestamp);
        }

		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}
