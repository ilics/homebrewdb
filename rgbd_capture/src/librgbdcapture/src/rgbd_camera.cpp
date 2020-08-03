//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "rgbd_camera.h"
#include "openni_camera.h"
#include "realsense_camera.h"
#include "kinectv2_camera.h"
#include "kinect_azure_camera.h"

#define _WIN32_WINNT 0x0600


RGBDCameraAPI RGBDCameraInterfaceHandle get_rgbd_camera(const std::string &camera_name)
{
	if (camera_name == "kinect")
	{
		return new OpenNICamera();
	}
	if (camera_name == "kinect2")
	{
		return new KinectV2Camera();
	}
	if (camera_name == "realsense")
	{
		return new RealSenseCamera();
	}
	if (camera_name == "kinect_azure")
	{
		return new KinectAzureCamera();
	}
	
	std::cerr << "ERROR: unknown camera: " << camera_name << std::endl;

	return nullptr;
}
