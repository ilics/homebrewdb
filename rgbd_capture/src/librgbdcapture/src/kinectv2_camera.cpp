//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "kinectv2_camera.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <ppl.h>
#include "rgbd_camera.h"
#include <math_tools.h>

using namespace Microsoft::WRL;
using namespace std;
using namespace cv;

KinectV2Camera::KinectV2Camera()
{
	initialize_sensor();
	initialize_color();
	initialize_depth();

	// wait for init
	this_thread::sleep_for(chrono::seconds(2));

	m_is_running = true;

	for (int i = 0; i < 30; i++) get_frames_live();
}

void KinectV2Camera::initialize_sensor()
{
	ERROR_CHECK(GetDefaultKinectSensor(&sensor));
	ERROR_CHECK(sensor->Open());

	BOOLEAN is_open = FALSE;
	ERROR_CHECK(sensor->get_IsOpen(&is_open));

	if (!is_open)
	{
		throw runtime_error("Failed to open kinect sensor");
	}

	ERROR_CHECK(sensor->get_CoordinateMapper(&coordinate_mapper));
}

void KinectV2Camera::initialize_color()
{
	// Open Color Reader
	ComPtr<IColorFrameSource> color_frame_source;
	ERROR_CHECK(sensor->get_ColorFrameSource(&color_frame_source));
	ERROR_CHECK(color_frame_source->OpenReader(&color_frame_reader));

	// Retrieve Color Description
	ComPtr<IFrameDescription> color_frame_description;

	ERROR_CHECK(color_frame_source->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &
		color_frame_description));
	ERROR_CHECK(color_frame_description->get_Width(&color_frame_width));
	ERROR_CHECK(color_frame_description->get_Height(&color_frame_height));
	ERROR_CHECK(color_frame_description->get_BytesPerPixel(&color_bytes_per_pixel));

	float horizontal_field_of_view;
	float vertical_field_of_view;

	ERROR_CHECK(color_frame_description->get_HorizontalFieldOfView(&horizontal_field_of_view));
	ERROR_CHECK(color_frame_description->get_VerticalFieldOfView(&vertical_field_of_view));

	float fx = tr::fovToFocalLength(tr::degrees_to_radians(horizontal_field_of_view), color_frame_width);
	float fy = tr::fovToFocalLength(tr::degrees_to_radians(vertical_field_of_view), color_frame_height);
	float px = static_cast<float>(color_frame_width) / 2.0f;
	float py = static_cast<float>(color_frame_height) / 2.0f;

	intrinsics = IntrinsicParameters(fx, px, fy, py);

	color_buffer.resize(color_frame_height * color_frame_width * color_bytes_per_pixel);
}

void KinectV2Camera::initialize_depth()
{
	ComPtr<IDepthFrameSource> depth_frame_source;
	ERROR_CHECK(sensor->get_DepthFrameSource(&depth_frame_source));
	ERROR_CHECK(depth_frame_source->OpenReader(&depth_frame_reader));

	// Retrieve Depth Description
	ComPtr<IFrameDescription> depth_frame_description;
	ERROR_CHECK(depth_frame_source->get_FrameDescription(&depth_frame_description));
	ERROR_CHECK(depth_frame_description->get_Width(&depth_frame_width));
	ERROR_CHECK(depth_frame_description->get_Height(&depth_frame_height));
	ERROR_CHECK(depth_frame_description->get_BytesPerPixel(&depth_bytes_per_pixel));

	depth_buffer.resize(depth_frame_width * depth_frame_height);
}

KinectV2Camera::~KinectV2Camera()
{
	if (sensor)
	{
		sensor->Close();
	}
}

bool KinectV2Camera::get_color_frame()
{
	ComPtr<IColorFrame> color_frame;
	const HRESULT ret = color_frame_reader->AcquireLatestFrame(&color_frame);
	if (FAILED(ret))
	{
		cout << "\n\nFailed to capture color frame\n\n\n\n" << endl;
		return false;
	}

	ERROR_CHECK(color_frame->CopyConvertedFrameDataToArray(static_cast<UINT>(color_buffer.size()), &color_buffer[0],
		ColorImageFormat::ColorImageFormat_Bgra));
	return true;
}

bool KinectV2Camera::get_depth_frame()
{
	ComPtr<IDepthFrame> depth_frame;
	const HRESULT ret = depth_frame_reader->AcquireLatestFrame(&depth_frame);
	if (FAILED(ret))
	{
		cout << "\n\nFailed to capture depth frame\n\n\n\n" << endl;
		return false;
	}

	ERROR_CHECK(depth_frame->CopyFrameDataToArray(static_cast<UINT>(depth_buffer.size()), &depth_buffer[0]));
	return true;
}

void KinectV2Camera::map_frames()
{
	vector<DepthSpacePoint> depth_space_points(color_frame_width * color_frame_height);
	ERROR_CHECK(coordinate_mapper->MapColorFrameToDepthSpace(depth_buffer.size(), &depth_buffer[0], depth_space_points.
		size(), &depth_space_points[0]));

	vector<UINT16> buffer(color_frame_width * color_frame_height);

	Concurrency::parallel_for(0, color_frame_height, [&](const int color_y)
	{
		const unsigned int color_offset = color_y * color_frame_width;
		for (int color_x = 0; color_x < color_frame_width; color_x++)
		{
			const unsigned int color_index = color_offset + color_x;
			const int depth_x = static_cast<int>(round(depth_space_points[color_index].X));
			const int depth_y = static_cast<int>(round(depth_space_points[color_index].Y));
			if ((0 <= depth_x) && (depth_x < depth_frame_width) && (0 <= depth_y) && (depth_y < depth_frame_height))
			{
				const unsigned int depthIndex = depth_y * depth_frame_width + depth_x;
				buffer[color_index] = depth_buffer[depthIndex];
			}
		}
	});

	depth = Mat(color_frame_height, color_frame_width, CV_16UC1, &buffer[0]).clone();
	bgr_mat = Mat(color_frame_height, color_frame_width, CV_8UC4, &color_buffer[0]);
}


bool KinectV2Camera::get_frames_live()
{
	if (!m_is_running) return false;

	if (!(get_color_frame() && get_depth_frame()))
	{
		return false;
	}

	map_frames();

	if (depth.empty() || bgr_mat.empty())
	{
		return false;
	}

	cvtColor(bgr_mat, bgr_mat, CV_BGRA2BGR);
	flip(bgr_mat, bgr_mat, 1);

	cvtColor(bgr_mat, rgb_mat, CV_BGR2RGB);
	flip(depth, depth, 1);

	depth.convertTo(depth_float, CV_32FC1, 0.001);
	return true;
}
