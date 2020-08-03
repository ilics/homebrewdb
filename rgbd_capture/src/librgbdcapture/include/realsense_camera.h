//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H
#include "rgbd_camera.h"
#include <iostream>
#include <iomanip>
#include <librealsense2/rs.hpp>

static const int FPS = 30;

class RealSenseCamera: public RGBDCamera
{
public:
	RealSenseCamera();

	~RealSenseCamera();

	bool get_frames_live();

private:
	rs2::pipeline pipeline;
	rs2::align align;

	rs2::decimation_filter dec_filter;
	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;

	// rs2::disparity_transform *depth_to_disparity;
	// rs2::disparity_transform *disparity_to_depth;

	rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
	rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);

	float depth_scale;

	rs2::depth_sensor get_depth_sensor(const rs2::device &device);
};

#endif
