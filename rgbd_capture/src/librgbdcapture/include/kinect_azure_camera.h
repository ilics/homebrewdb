//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "rgbd_camera.h"
#include <k4a/k4a.h>


class KinectAzureCamera: public RGBDCamera
{
public:
	KinectAzureCamera();
	~KinectAzureCamera();
	bool get_frames_live();

private:
	void initialize_sensor();

	void initialize_color();

	void initialize_depth();

private:
	k4a_device_t device = NULL;
	k4a_calibration_t calibration;
	k4a_device_configuration_t config;
	k4a_transformation_t transformation = NULL;


	const int32_t CAPTURE_TIMEOUT_IN_MS = 1000;



	
};