//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef RGBD_OPENNI_CAMERA
#define RGBD_OPENNI_CAMERA

#include "OpenNI.h"
#include <opencv/cv.hpp>
#include <iostream>

#include "rgbd_camera.h"

	class OpenNICamera: public RGBDCamera
	{
	public:
		OpenNICamera();

		~OpenNICamera();

	    bool get_frames_live();
	private:
		void init_for_live_capture();
		bool init_intrinsics();


		openni::VideoStream** streams;
		openni::Device device;
		openni::VideoStream color_stream;
		openni::VideoStream depth_stream;

		openni::VideoFrameRef depth_frame;
		openni::VideoFrameRef color_frame;

		bool mirror_frames = false;
		float depth_scale = 0.001;
	};

#endif
