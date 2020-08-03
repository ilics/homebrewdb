//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "realsense_camera.h"
using namespace std;
using namespace cv;

RealSenseCamera::RealSenseCamera(): RGBDCamera(1280, 720), align(RS2_STREAM_COLOR)
{
	rs2::config config;

	depth_to_disparity = new rs2::disparity_transform(true);
	disparity_to_depth = new rs2::disparity_transform(false);

	config.enable_stream(RS2_STREAM_DEPTH, depth_frame_width, depth_frame_height, RS2_FORMAT_Z16, FPS);
	config.enable_stream(RS2_STREAM_COLOR, color_frame_width, color_frame_height, RS2_FORMAT_RGB8, FPS);

	auto profile = pipeline.start(config);

	rs2::depth_sensor depth_sensor = get_depth_sensor(profile.get_device());

	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

	depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, 0.0001);
	//depth_sensor.set_option(RS2_OPTION_LASER_POWER, 30.0);

	// because of the alignment
	depth_scale = depth_sensor.get_depth_scale();
	m_is_running = true;

	for (int i = 0; i < 10; ++i) get_frames_live();

	auto c_intrinsics = color_stream.get_intrinsics();
	intrinsics = IntrinsicParameters(c_intrinsics.fx, c_intrinsics.ppx, c_intrinsics.fy, c_intrinsics.ppy);
}

RealSenseCamera::~RealSenseCamera()
{
	if (m_is_running)
	{
		pipeline.stop();
	}
}

rs2::depth_sensor RealSenseCamera::get_depth_sensor(const rs2::device& device)
{
	for (rs2::sensor& sensor : device.query_sensors())
	{
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt;
		}
	}
	throw runtime_error("Device does not have a depth sensor");
}

bool RealSenseCamera::get_frames_live()
{
	if (!m_is_running) return false;

	auto frames = pipeline.wait_for_frames();
	auto aligned_frames = align.process(frames);

	auto depth_frame = aligned_frames.get_depth_frame();
	auto color_frame = aligned_frames.get_color_frame();

	auto processed_depth = depth_frame;

#ifdef REFINE_DEPTH

		processed_depth = dec_filter.process(processed_depth);
		processed_depth = depth_to_disparity.process(processed_depth);
		processed_depth = spat_filter.process(processed_depth);
		processed_depth = temp_filter.process(processed_depth);
		processed_depth = disparity_to_depth.process(processed_depth);
#endif

	rgb_mat = Mat(Size(color_frame_width, color_frame_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

#ifdef REFINE_DEPTH
		Mat depth_temp = Mat(Size(processed_depth.get_width(), processed_depth.get_height()), CV_16UC1, (void*)processed_depth.get_data(), Mat::AUTO_STEP);
		resize(depth_temp, depth, Size(depth_frame_width, depth_frame_height));
#else

	depth = Mat(Size(processed_depth.get_width(), processed_depth.get_height()), CV_16UC1,
	            (void*)processed_depth.get_data(), Mat::AUTO_STEP);
#endif

	cvtColor(rgb_mat, bgr_mat, CV_RGB2BGR);
	depth.convertTo(depth_float, CV_32FC1, depth_scale);

	return true;
}
