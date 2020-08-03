//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "kinect_azure_camera.h"
#include <iostream>

using namespace std;

#define ERROR_CHECK(ret, error_message)                        \
    if (K4A_RESULT_SUCCEEDED != ret) {                                          \
        std::stringstream ss;                                     \
        ss << "failed " #ret " " << std::hex << ret << " " << error_message << std::endl; \
        throw std::runtime_error( ss.str().c_str() );             \
    }

KinectAzureCamera::KinectAzureCamera()
{
	initialize_sensor();
	initialize_color();
	initialize_depth();

	for (int i = 0; i < 10; ++i) get_frames_live();
}

void KinectAzureCamera::initialize_sensor()
{
	uint32_t device_count = k4a_device_get_installed_count();
	if (device_count != 1)
	{
		cerr << "More than one device connected" << endl;
		throw std::runtime_error("Error[Sensor]: more than one device connected\n");
	}

	ERROR_CHECK(k4a_device_open(K4A_DEVICE_DEFAULT, &device), "Failed to open device")

	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
	config.synchronized_images_only = true;

	ERROR_CHECK(k4a_device_start_cameras(device, &config), "Failed to start device")
	ERROR_CHECK(k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration), "Failed to get calibration")
	transformation = k4a_transformation_create(&calibration);
}

void KinectAzureCamera::initialize_color()
{
	color_frame_width = calibration.color_camera_calibration.resolution_width;
	color_frame_height = calibration.color_camera_calibration.resolution_height;

	const auto &calibration_params = calibration.color_camera_calibration.intrinsics.parameters.param;
	intrinsics = IntrinsicParameters(calibration_params.fx, calibration_params.cx, calibration_params.fy, calibration_params.cy);
}

void KinectAzureCamera::initialize_depth()
{
	depth_frame_height = calibration.depth_camera_calibration.resolution_height;
	depth_frame_width = calibration.depth_camera_calibration.resolution_width;
}


bool KinectAzureCamera::get_frames_live()
{
	k4a_capture_t capture;
	auto ret_code = k4a_device_get_capture(device, &capture, CAPTURE_TIMEOUT_IN_MS);

	if (ret_code == K4A_WAIT_RESULT_SUCCEEDED)
	{
		k4a_image_t color_image = k4a_capture_get_color_image(capture);

		if (!color_image)
		{
			cerr << "Failed to capture color image" << endl;
			k4a_capture_release(capture);
			return false;
		}

		k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
		
		if (!depth_image)
		{
			cerr << "Failed to capture depth image" << endl;
			k4a_capture_release(capture);
			return false;
		}

		k4a_image_t transformed_depth_image;
		ERROR_CHECK(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			color_frame_width,
			color_frame_height,
			color_frame_width * (int)sizeof(uint16_t),
			&transformed_depth_image), "Failed to create registered depth image");

		ERROR_CHECK(k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image),
			"Failed to register images")

		uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(transformed_depth_image));
		uint8_t *color_buffer = k4a_image_get_buffer(color_image);

		cv::Mat(color_frame_height, color_frame_width, CV_16UC1, depth_buffer).copyTo(depth);
		cv::Mat(color_frame_height, color_frame_width, CV_8UC4, color_buffer).copyTo(bgr_mat);

		cvtColor(bgr_mat, bgr_mat, CV_BGRA2BGR);
		cvtColor(bgr_mat, rgb_mat, CV_BGR2RGB);

		depth.convertTo(depth_float, CV_32FC1, 0.001);

		k4a_image_release(depth_image);
		k4a_image_release(color_image);
		k4a_image_release(transformed_depth_image);

		k4a_capture_release(capture);

		return true;		
	}
	if (ret_code == K4A_WAIT_RESULT_TIMEOUT)
	{
		cerr << "Timed out waiting for a capture" << endl;
		return false;
	} 
	
	throw std::runtime_error("Failed to read a capture\n");
	
}


KinectAzureCamera::~KinectAzureCamera()
{
	if (device != NULL)
	{
		k4a_device_close(device);
	}

}

