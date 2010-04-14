///////////////////////////////////////////////////////////////////////////////
// this program just uses sicktoolbox to get laser scans, and then publishes
// them as ROS messages
//
// Copyright (C) 2008, Morgan Quigley
//
// I am distributing this code under the BSD license:
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <csignal>
#include <cstdio>
#include <sicklms-1.0/SickLMS.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
using namespace SickToolbox;
using namespace std;

void publish_scan(ros::Publisher *pub, uint32_t *values, uint32_t num_values, 
		double scale, ros::Time start, double scan_time, bool inverted, std::string frame_id)
{
  static int scan_count = 0;
  static double last_print_time = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  ros::Time t = start;
  double t_d = t.toSec();
  if (t_d > last_print_time + 1)	{
    last_print_time = t_d;
//    printf("publishing scan %d, inverted=%d\n", scan_count, inverted);
  }
  if (inverted) {
    scan_msg.angle_min = M_PI/2;
    scan_msg.angle_max = -M_PI/2;
  } else {
    scan_msg.angle_min = -M_PI/2;
    scan_msg.angle_max = M_PI/2;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(num_values-1);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(num_values-1);
  scan_msg.range_min = 0;
  if (scale == 0.01) {
    scan_msg.range_max = 81;
  }
  else if (scale == 0.001) {
    scan_msg.range_max = 8.1;
  }
  //	scan_msg.set_ranges_size(num_values);
  scan_msg.ranges.resize(num_values);
  //  scan_msg.intensities.resize(num_values);
  scan_msg.header.stamp = t;
  for (size_t i = 0; i < num_values; i++) {
    scan_msg.ranges[i] = (float)values[i] * (float)scale;
    //    scan_msg.intensities[i] = 0;
  }
  /*

  static double prev_time = 0;
  double cur_time = ros::Time::now().toSec();
  if (rand() % 10 == 0)
  printf("%f\n", cur_time - prev_time);
  prev_time = cur_time;
  */
  pub->publish(scan_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sicklms");
	string port;
	int baud;
	bool inverted;
	int angle;
	double resolution;
	std::string frame_id;
	double scan_time = 0;

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
	nh_ns.param("port", port, string("/dev/lms200"));
	nh_ns.param("baud", baud, 38400);
	nh_ns.param("inverted", inverted, false);
	nh_ns.param("angle", angle, 0);
	nh_ns.param("resolution", resolution, 0.0);
	nh_ns.param<std::string>("frame_id", frame_id, "laser");

	SickLMS::sick_lms_baud_t desired_baud = SickLMS::IntToSickBaud(baud);
	if (desired_baud == SickLMS::SICK_BAUD_UNKNOWN)
	{
		ROS_ERROR("Baud rate must be in {9600, 19200, 38400, 500000}");
		return 1;
	}
	uint32_t values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
	uint32_t num_values = 0;
	SickLMS sick_lms(port);
	double scale = 0;

	try
	{
		sick_lms.Initialize(desired_baud);
		if (angle == 0)
                  angle = sick_lms.GetSickScanAngle();
                if (resolution == 0.0)
                  resolution = sick_lms.GetSickScanResolution();
                sick_lms.SetSickVariant(sick_lms.IntToSickScanAngle(angle),
			sick_lms.DoubleToSickScanResolution(resolution));
                ROS_INFO("Setting variant to (%i, %f)", angle, resolution);
		SickLMS::sick_lms_measuring_units_t u = sick_lms.GetSickMeasuringUnits();
		if (u == SickLMS::SICK_MEASURING_UNITS_CM)
			scale = 0.01;
		else if (u == SickLMS::SICK_MEASURING_UNITS_MM)
			scale = 0.001;
		else
		{
			ROS_ERROR("Bogus measuring unit.");
			return 1;
		}

		double scan_res = sick_lms.GetSickScanResolution();
		SickLMS::sick_lms_scan_resolution_t scan_resolution = SickLMS::DoubleToSickScanResolution(scan_res);

		if ( scan_resolution == SickLMS::SICK_SCAN_RESOLUTION_25) {  // 0.25 degrees
			scan_time = 4.0 / 75;   // 53.33 ms
		}
		else if ( scan_resolution == SickLMS::SICK_SCAN_RESOLUTION_50) {  // 0.5 degrees
			scan_time = 2.0 / 75;   // 26.66 ms
		}
		else if ( scan_resolution == SickLMS::SICK_SCAN_RESOLUTION_100) { // 1 degree
			scan_time = 1.0 / 75;   // 13.33 ms
		}
		else {
			ROS_ERROR("Bogus scan resolution.");
			return 1;
		}
	}
	catch (...)
	{
		ROS_ERROR("Initialise failed! are you using the correct device path?");
	}
	try
	{
		while (ros::ok())
		{
			ros::Time start = ros::Time::now();
			sick_lms.GetSickScan(values, num_values);
			publish_scan(&scan_pub, values, num_values, scale, start, scan_time, inverted, frame_id);
			ros::spinOnce();
		}
	}
	catch (...)
	{
		ROS_ERROR("woah! error!");
		return 1;
	}
	try
	{
		sick_lms.Uninitialize();
	}
	catch (...)
	{
		ROS_ERROR("error during uninitialize");
		return 1;
	}
	ROS_INFO("success.\n");

	return 0;
}

