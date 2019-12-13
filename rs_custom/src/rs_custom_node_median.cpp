// https://github.com/IntelRealSense/librealsense/wiki/API-How-To
// https://github.com/IntelRealSense/librealsense/blob/master/examples/capture/rs-capture.cpp

#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include <librealsense2/rs.hpp> 
//#include <opencv2/opencv.hpp>   
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs_advanced_mode.hpp>
#include <ros/console.h>
#include <algorithm>

using std::cout;
using std::endl;

int main(int argc, char * argv[]) try
{
    // Set up ROS
    ros::init(argc, argv, "rs_custom_node");
    ros::NodeHandle nh_("~"); 

	// Set publishers
    ros::Publisher pub_depth = nh_.advertise<std_msgs::Float32MultiArray>("/depth_row", 1);

    // Pre define parameters
    int DEPTH_WIDTH = 640;
    int DEPTH_HEIGHT = 480;
    int DEPTH_FPS = 30;
	int N_ROWS = 3;
	int DN_ROWS = 5;
    
    nh_.param("depth_width", DEPTH_WIDTH, DEPTH_WIDTH);
    nh_.param("depth_height", DEPTH_HEIGHT, DEPTH_HEIGHT);
    nh_.param("depth_fps", DEPTH_FPS, DEPTH_FPS);
	nh_.param("n_rows", N_ROWS, N_ROWS);
	nh_.param("dn_rows", DN_ROWS, DN_ROWS);
     
    //ROS_INFO("fps %i",DEPTH_FPS);

    // Declare RealSense pipeline and config
    rs2::pipeline pipe;	
	rs2::config cfg;

	// Configure depth stream
	cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FPS);

	// Start streaming with specified configuration
	rs2::pipeline_profile selection = pipe.start(cfg);
	ROS_INFO("[REALSENSE] Streaming pipeline started");

    // Find first depth sensor (devices can have zero or more then one)
	auto device = selection.get_device();
	auto depth_sensor = device.first<rs2::depth_sensor>();
    auto scale =  depth_sensor.get_depth_scale();

	// Controlling the laser
	//if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
	//{
	//	depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
		// depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	//}
	//if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
	//{
		// Query min and max values:
	//	auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
	//	depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
		// depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
	//}
	
	using namespace cv;
    // Pre-define depth matrix size
    Mat depth_matrix_m(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_32FC1);
	// Define depth msg 
	std_msgs::Float32MultiArray depth_msg;
	std::vector<int> vals;
	std::vector<int> depth_row;

	bool first_loop = true;
	while (ros::ok())
    {
        if(first_loop)
        {
            cout << "Entering loop" << endl;
            first_loop = false;
        }

        
        // Wait for next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames(4000);
		rs2::frame frame = frames.get_depth_frame();

        Mat depth_matrix(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_16U, (void*)(frame.get_data()), Mat::AUTO_STEP);

               
        // copy in the data
		depth_msg.data.clear();
		for (int ix=0; ix<DEPTH_WIDTH; ix++){
			// Define value
			vals.clear();

			for (int iy=0; iy<N_ROWS; iy++){
				if (depth_matrix.at<int>(Point(ix,DEPTH_HEIGHT/2 - iy*DN_ROWS))!=0.0){
					vals.push_back(depth_matrix.at<int>(Point(ix,DEPTH_HEIGHT/2 - iy*DN_ROWS)));
				}
			}

			if (vals.size() == 0){
				depth_msg.data.push_back(0.0);
			} else {
				float* first(&vals[0]);
				float* last(first + 4);
				std::sort(first, last);
				depth_msg.data.push_back(vals[static_cast<int>(vals.size()/2)]);
			}
		}

		// Convert from 16b to meters
        depth_matrix.convertTo(depth_matrix_m, CV_32F, scale);
         
/*		Mat depth_row(Size(DEPTH_WIDTH, 0), CV_32F);
=======
		//for (int ix=0; ix<DEPTH_WIDTH; ix++){
			// Define value
		//	vals.clear();

			//cout << "boobs" << endl;
		//	for (int iy=0; iy<N_ROWS; iy++){
		//		ROS_INFO("the thing %f", depth_matrix_m.at<float>(ix,DEPTH_HEIGHT/2 - iy*DN_ROWS));
		//		cout << depth_matrix_m << endl;
		//		if (depth_matrix_m.at<float>(ix,DEPTH_HEIGHT/2 - iy*DN_ROWS)!=0.0){
		//			vals.push_back(depth_matrix_m.at<float>(ix,DEPTH_HEIGHT/2 - iy*DN_ROWS));
		//		}
				//cout << "balls" << endl;
		//	}
		//	depth_msg.data.push_back(std::accumulate(vals.begin(),vals.end(),0.0)/vals.size());
			//ROS_INFO("Vals begin %f",vals.end());
		//	ROS_INFO("IX %i",ix);
		//}
		cv::Mat depth_row(Size(DEPTH_WIDTH, 0), CV_32F);
>>>>>>> b3d33bc610d80c5d83b6a6dbe5851b8287471114
        depth_row.push_back(depth_matrix_m.row(DEPTH_HEIGHT/2));
		
        depth_msg.data.clear();
        //depth_msg.data.insert(depth_msg.data.end(), (float*)depth_matrix_m.datastart, (float*)depth_matrix_m.dataend);
		depth_msg.data.insert(depth_msg.data.end(), (float*)depth_row.datastart, (float*)depth_row.dataend);
<<<<<<< HEAD
*/
		pub_depth.publish(depth_msg);

	}

    return EXIT_SUCCESS;

} // end main()

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
    
