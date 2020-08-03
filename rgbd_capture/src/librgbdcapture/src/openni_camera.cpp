//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "openni_camera.h"
#include "math_tools.h"
#include <iomanip>

using namespace std;
using namespace cv;

OpenNICamera::OpenNICamera(): RGBDCamera(640, 480)
{
	depth = Mat(depth_frame_height, depth_frame_width, CV_16UC1);
	depth_float = Mat(depth_frame_height, depth_frame_width, CV_32FC1);

	rgb_mat = Mat(color_frame_height, color_frame_width, CV_8UC3);
	bgr_mat = Mat(color_frame_height, color_frame_width, CV_8UC3);

	init_for_live_capture();
}

bool OpenNICamera::init_intrinsics()
{
	float horizontal_field_of_view;
	float vertical_field_of_view;

	if (color_stream.getProperty<float>(openni::STREAM_PROPERTY_HORIZONTAL_FOV, &horizontal_field_of_view) != openni::
		STATUS_OK)
	{
		cerr << "Error [Color]: Failed to obtain intrinsics from stream" << endl;
		return false;
	}

	if (color_stream.getProperty<float>(openni::STREAM_PROPERTY_VERTICAL_FOV, &vertical_field_of_view) != openni::
		STATUS_OK)
	{
		cerr << "Error [Color]: Failed to obtain intrinsics from stream" << endl;
		return false;
	}

	float fx = tr::fovToFocalLength(horizontal_field_of_view, color_frame_width);
	float fy = tr::fovToFocalLength(vertical_field_of_view, color_frame_height);
	float px = static_cast<float>(color_frame_width) / 2.0f;
	float py = static_cast<float>(color_frame_height) / 2.0f;

	intrinsics = IntrinsicParameters(fx, px, fy, py);
	return true;
}

void OpenNICamera::init_for_live_capture()
{
	openni::OpenNI::initialize();
	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
	device.open(openni::ANY_DEVICE);

	if (device.isValid())
	{
		const openni::SensorInfo* depth_sensor_info = device.getSensorInfo(openni::SENSOR_DEPTH);
		const openni::Array<openni::VideoMode>& depth_video_modes = depth_sensor_info->getSupportedVideoModes();
		const openni::SensorInfo* color_sensor_info = device.getSensorInfo(openni::SENSOR_COLOR);
		const openni::Array<openni::VideoMode>& color_video_modes = color_sensor_info->getSupportedVideoModes();

		int color_mode_index = -1;
		for (int i = 0; i < color_video_modes.getSize(); i++)
		{
			const openni::VideoMode& color_mode = color_video_modes[i];
			cout << "Color [" << i << "]: " << color_mode.getResolutionX() << "x" << color_mode.getResolutionY() <<
				", fps = " << color_mode.getFps() << endl;
			if (color_mode.getResolutionX() == color_frame_width && color_mode.getResolutionY() == color_frame_height
				&& color_mode.getFps() == 30 && color_mode.getPixelFormat() == openni::PIXEL_FORMAT_RGB888)
			{
				color_mode_index = i;
				break;
			}
		}

		int depth_mode_index = -1;
		for (int i = 0; i < depth_video_modes.getSize(); i++)
		{
			const openni::VideoMode& depth_mode = depth_video_modes[i];

			cout << "Depth [" << i << "]: " << depth_mode.getResolutionX() << "x" << depth_mode.getResolutionY() <<
				", fps = " << depth_mode.getFps() << endl;
			if (depth_mode.getResolutionX() == depth_frame_width && depth_mode.getResolutionY() == depth_frame_height
				&& depth_mode.getFps() == 30)
			{
				depth_mode_index = i;

				if (depth_mode.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_100_UM)
				{
					//favor finer depth
					depth_scale *= 0.1f;
					break;
				}
			}
		}

		if (depth_mode_index < 0 || color_mode_index < 0)
		{
			cerr << "Error[Sensor]: No suitable mode with 640x480." << endl;
			m_is_running = false;
			return;
		}

		depth_stream.create(device, openni::SENSOR_DEPTH);
		openni::Status res = depth_stream.setVideoMode(depth_video_modes[depth_mode_index]);

		if (res != openni::STATUS_OK)
		{
			cerr << "Error[Depth]: Video Mode is not supported in OpenNI." << endl;
		}
		else
		{
			color_stream.create(device, openni::SENSOR_COLOR);
			res = depth_stream.setVideoMode(color_video_modes[color_mode_index]);

			if (res != openni::STATUS_OK)
			{
				cerr << "Error[Color]: Video Mode is not supported in OpenNI." << endl;
			}
			else
			{
				if (color_stream.isPropertySupported(openni::STREAM_PROPERTY_MIRRORING) && depth_stream.
					isPropertySupported(openni::STREAM_PROPERTY_MIRRORING))
				{
					depth_stream.setMirroringEnabled(false);
					color_stream.setMirroringEnabled(false);
				}
				else
				{
					mirror_frames = true;
				}

				if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
				{
					device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
					cout << "[Sensor]: Depth to color registration enabled." << endl;
				}
				else
				{
					cout << "[Sensor]: Failed to enable depth to color registration." << endl;
				}

				depth_stream.start();
				color_stream.start();

				init_intrinsics();

				streams = new openni::VideoStream*[2];
				streams[0] = &depth_stream;
				streams[1] = &color_stream;

				openni::Status sync_enabled = device.setDepthColorSyncEnabled(true);
				cout << "[Sensor]: Depth and color sync " << (sync_enabled == openni::STATUS_OK
					                                              ? "enabled."
					                                              : "could not be enabled.") << endl;

				m_is_running = true;
			}
		}


		for (int i = 0; i < 10; ++i) this->get_frames_live();
	}
	else
	{
		cerr << "Error[Sensor]: No camera." << endl;
		getchar();
		openni::OpenNI::shutdown();
		exit(-1);
	}
}

OpenNICamera::~OpenNICamera()
{
	if (m_is_running)
	{
		if (device.isValid())
		{
			depth_stream.destroy();
			color_stream.destroy();

			device.close();
		}
		openni::OpenNI::shutdown();
	}
}


bool OpenNICamera::get_frames_live()
{
	if (!m_is_running) return false;

	int changed_index;

	openni::Status rc = openni::OpenNI::waitForAnyStream(streams, 2, &changed_index);

	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return false;
	}

	depth_stream.readFrame(&depth_frame);
	color_stream.readFrame(&color_frame);

	memcpy(depth.data, depth_frame.getData(), depth_frame_width * depth_frame_height * sizeof(unsigned short));
	memcpy(rgb_mat.data, color_frame.getData(), 3 * color_frame_width * color_frame_height * sizeof(unsigned char));

	if (mirror_frames)
	{
		flip(rgb_mat, rgb_mat, 1);
		flip(depth, depth, 1);
	}

	cvtColor(rgb_mat, bgr_mat, CV_RGB2BGR); // opencv: the image is in BGR format
	depth.convertTo(depth_float, CV_32FC1, depth_scale);
	return true;
}
