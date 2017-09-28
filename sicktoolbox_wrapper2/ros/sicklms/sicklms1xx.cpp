///////////////////////////////////////////////////////////////////////////////
// this program just uses sicktoolbox to get laser scans, and then publishes
// them as ROS messages
//
// Copyright (C) 2008, Morgan Quigley
// Copyright (C) 2011, Robert Huitl
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
#include <math.h>
#include <sicktoolbox2/SickLMS1xx.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
using namespace SickToolbox;
using namespace std;

void publish_scan(ros::Publisher *pub, uint32_t *range_values,
                  uint32_t n_range_values, uint32_t *intensity_values,
                  uint32_t n_intensity_values, double scale, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id)
{
	static int scan_count = 0;
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.frame_id = frame_id;
	scan_count++;
	if(inverted) {
		scan_msg.angle_min = angle_max;
		scan_msg.angle_max = angle_min;
	} else {
		scan_msg.angle_min = angle_min;
		scan_msg.angle_max = angle_max;
	}
	scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)
	                / (double)(n_range_values - 1);

	scan_msg.scan_time = scan_time;
	scan_msg.time_increment = scan_time / (2 * M_PI) * scan_msg.angle_increment;
	scan_msg.range_min = 0;
	scan_msg.range_max = 21.;      // Note: change to 51. for the LMS 151
	scan_msg.ranges.resize(n_range_values);
	scan_msg.header.stamp = start;
	for(size_t i = 0; i < n_range_values; i++) {
		scan_msg.ranges[i] = (float)range_values[i] * (float)scale;
	}
	scan_msg.intensities.resize(n_intensity_values);
	for(size_t i = 0; i < n_intensity_values; i++) {
		scan_msg.intensities[i] = (float)intensity_values[i];
	}

	pub->publish(scan_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sicklms1xx");
	string port;
	int baud;
	bool inverted = false;
	int angle;
	double resolution;
	std::string frame_id;
	std::string ip_add;
	double scan_time = 0;
	double angle_increment = 0;
	float angle_min = 0.0;
	float angle_max = 0.0;

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan> ("scan", 1);
	//nh_ns.param("port", port, string("/dev/lms200"));
	//nh_ns.param("baud", baud, 38400);
	//nh_ns.param("inverted", inverted, false);
	nh_ns.param("angle", angle, 0);
	nh_ns.param("resolution", resolution, 0.0);
	nh_ns.param<std::string>("ip_add", ip_add, "192.168.0.1");
	nh_ns.param<std::string> ("frame_id", frame_id, "laser");

	uint32_t range_values[SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS] = { 0 };
	uint32_t intensity_values[SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS] = { 0 };
	uint32_t n_range_values = 0;
	uint32_t n_intensity_values = 0;
	SickLMS1xx sick_lms(ip_add);
	double scale = 0;
	double angle_offset;
	uint32_t partial_scan_index;

	try {
		sick_lms.Initialize();
		sick_lms.SetSickScanDataFormat(SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_NONE);
		sick_lms.SetSickScanFreqAndRes(SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_25,
		                               SickLMS1xx::SICK_LMS_1XX_SCAN_RES_25);
		/*sick_lms.SetSickScanFreqAndRes(SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_50,
		                               SickLMS1xx::SICK_LMS_1XX_SCAN_RES_50);*/

		// Scale is mm with the 1xx driver
		scale = 0.001;

		/*
		// The scan time is always 1/75 because that's how long it takes
		// for the mirror to rotate. If we have a higher resolution, the
		// SICKs interleave the readings, so the net result is we just
		// shift the measurements.
		SickLMS1xx::sick_lms_2xx_scan_resolution_t scan_resolution =
						SickLMS1xx::DoubleToSickScanResolution(resolution);
		if(scan_resolution == SickLMS1xx::SICK_SCAN_RESOLUTION_25) {
			// 0.25 degrees
			scan_time = 4.0 / 75; // 53.33 ms
		} else if(scan_resolution == SickLMS1xx::SICK_SCAN_RESOLUTION_50) {
			// 0.5 degrees
			scan_time = 2.0 / 75; // 26.66 ms
		} else if(scan_resolution == SickLMS1xx::SICK_SCAN_RESOLUTION_100) {
			// 1 degree
			scan_time = 1.0 / 75; // 13.33 ms
		} else {
			ROS_ERROR("Bogus scan resolution.");
			return 1;
		}
		if(scan_resolution != SickLMS1xx::SICK_SCAN_RESOLUTION_100) {
			ROS_WARN(
					 "You are using an angle smaller than 180 degrees and a "
						 "scan resolution less than 1 degree per scan. Thus, "
						 "you are in inteleaved mode and the returns will not "
						 "arrive sequentially how you read them. So, the "
						 "time_increment field will be misleading. If you need to "
						 "know when the measurement was made at a time resolution "
						 "better than the scan_time, use the whole 180 degree "
						 "field of view.");
		}

		// The increment for the slower LMS is still 1.0 even if its set to
		// 0.5 or 0.25 degrees resolution because it is interleaved. So for
		// 0.5 degrees, two scans are needed, offset by 0.5 degrees. These
		// show up as two seperate LaserScan messages.
		angle_increment = sick_lms.IsSickLMS1xxFast() ? 0.5 : 1.0;
		angle_offset = (180.0 - angle) / 2; */


	} catch(...) {
		ROS_ERROR("Initialize failed!");
		return 2;
	}
	try {
		ros::Time last_scan_time = ros::Time::now();
		while(ros::ok()) {
			angle_min = sick_lms.GetSickStartAngle() * M_PI / 180.0;
			angle_max = sick_lms.GetSickStopAngle()  * M_PI / 180.0;

			sick_lms.GetSickMeasurements(range_values, NULL, NULL, NULL, n_range_values);
			n_intensity_values = 0;


			// Figure out the time that the scan started. Since we just
			// fished receiving the data, we'll assume that the mirror is at
			// 180 degrees now, or half a scan time. In other words, we
			// assume a zero transfer time of the data.
			ros::Time end_of_scan = ros::Time::now();
			ros::Time start = end_of_scan; // TODO - ros::Duration(scan_time / 2.0);

			ros::Duration diff = start - last_scan_time;
			/*if(diff > ros::Duration(scan_time * 1.5)) {
				ROS_WARN_STREAM("A scan was probably missed. The last scan was "
				                << diff.toSec() << " seconds ago.");
			}*/
			last_scan_time = start;

			publish_scan(&scan_pub, range_values, n_range_values,
			             intensity_values, n_intensity_values, scale, start,
			             scan_time, inverted, angle_min, angle_max, frame_id);
			ros::spinOnce();
		}
	} catch(...) {
		ROS_ERROR("woah! error!");
		return 1;
	}
	try {
		sick_lms.Uninitialize();
	} catch(...) {
		ROS_ERROR("error during uninitialize");
		return 1;
	}
	ROS_INFO("success.\n");

	return 0;
}

