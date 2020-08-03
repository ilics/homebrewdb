//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef __RGBD_CAMERA_NTERFACE__H_
#define __RGBD_CAMERA_NTERFACE__H_
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

#define RGBDCameraLib_EXPORTS
#define NTDDI_VERSION NTDDI_VISTA 
#define _WIN32_WINNT _WIN32_WINNT_VISTA

#ifdef RGBDCameraLib_EXPORTS
#define RGBDCameraAPI __declspec(dllexport)
#else
#define RGBDCameraAPI __declspec(dllimport)
#endif

struct IntrinsicParameters
{
	float fx;
	float ox;
	float fy;
	float oy;

	IntrinsicParameters(float fx_, float ox_, float fy_, float oy_) : fx(fx_), ox(ox_), fy(fy_), oy(oy_) {}
	IntrinsicParameters(IntrinsicParameters const& other) : fx(other.fx), ox(other.ox), fy(other.fy), oy(other.oy) {}
	IntrinsicParameters() {}
};

class RGBDCameraInterface
{
public:
	virtual bool get_frames_live() = 0;

	virtual	const cv::Mat get_depth() const = 0;
	virtual	const cv::Mat get_depth_float() const = 0;
	virtual const cv::Mat get_rgb()   const = 0;
	virtual	const cv::Mat get_bgr()   const = 0;

	virtual bool is_running() const = 0;

	virtual IntrinsicParameters get_intrinsics() = 0;
};

typedef RGBDCameraInterface* RGBDCameraInterfaceHandle;

class RGBDCamera: public RGBDCameraInterface {
  public:

	RGBDCamera(int _width, int _height)
	{
		init(_width, _height);
	};

	RGBDCamera()
	{
		m_is_running = false;
	};

	~RGBDCamera(){};

    int color_frame_width;
	int color_frame_height;

	int depth_frame_width;
    int depth_frame_height;

	cv::Mat depth;
	cv::Mat depth_float;

	cv::Mat rgb_mat;
	cv::Mat bgr_mat;

	IntrinsicParameters intrinsics;

	bool is_running() const { return m_is_running; }

	IntrinsicParameters get_intrinsics()
	{
		return intrinsics;
	}

	const cv::Mat get_depth_float() const
	{
		return depth_float;
	}

	const cv::Mat get_depth() const
	{
		return depth;
	}

	const cv::Mat get_rgb() const
	{
		return rgb_mat;
	}

	const cv::Mat get_bgr() const
	{
		return bgr_mat;
	}

  protected:
	  bool m_is_running;

	  std::string base_folder;
  private:
	  void init(int width, int height)
	  {
		  color_frame_width = width;
		  color_frame_height = height;
		  depth_frame_width = width;
		  depth_frame_height = height;

		  m_is_running = false;
	  };
};

#ifdef __cplusplus
#   define EXTERN_C     extern "C"
#else
#   define EXTERN_C
#endif // __cplusplus

EXTERN_C
RGBDCameraAPI RGBDCameraInterfaceHandle get_rgbd_camera(const std::string &camera_name);
#endif