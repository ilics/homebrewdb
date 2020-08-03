//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef KINECTV2CAMERA_H
#define KINECTV2CAMERA_H


#include <opencv/cv.hpp>
#include <Kinect.h>
#include <rgbd_camera.h>

#include <sstream>
#include <stdexcept>

#include <Windows.h>
#include <opencv2/opencv.hpp>

#include <vector>

#include <wrl/client.h>


#define ERROR_CHECK(ret)                                        \
    if (FAILED(ret)) {                                          \
        std::stringstream ss;                                     \
        ss << "failed " #ret " " << std::hex << ret << std::endl; \
        throw std::runtime_error( ss.str().c_str() );             \
    }


	class KinectV2Camera: public RGBDCamera
	{
	public:
		KinectV2Camera();
		~KinectV2Camera();

		bool get_frames_live();

	private:
		Microsoft::WRL::ComPtr<IKinectSensor> sensor;
		Microsoft::WRL::ComPtr<ICoordinateMapper> coordinate_mapper;

		Microsoft::WRL::ComPtr<IColorFrameReader> color_frame_reader;
		Microsoft::WRL::ComPtr<IDepthFrameReader> depth_frame_reader;

		std::vector<BYTE> color_buffer;
		unsigned int color_bytes_per_pixel;

		std::vector<UINT16> depth_buffer;
		unsigned int depth_bytes_per_pixel;

		cv::Mat bgr_mat_tmp;
		
		void initialize_sensor();

		void initialize_color();

		void initialize_depth();

		bool get_color_frame();
		
		bool get_depth_frame();

		void map_frames();

		// Release function
		template< class T > void SafeRelease(T** ppT)
		{
			if (*ppT)
			{
				(*ppT)->Release();
				*ppT = nullptr;
			}
		}
	};
#endif