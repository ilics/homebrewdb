//######################################################################
//#   Liboffscreenrenderer Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef RENDERER_H
#define RENDERER_H

#include <Eigen/Core>
#include "model.h"
#include "painter.h"
#include <iostream>
#include <memory>

#ifdef _WIN32

#define RendererLib_EXPORTS

#ifdef RendererLib_EXPORTS
#define RendererAPI __declspec(dllexport)
#else
#define RendererAPI __declspec(dllimport)
#endif

#endif

struct RendererConfiguration
{
	int width;
	int height;

	Eigen::Matrix3f intrinsics;

	float z_near;
	float z_far;

	std::string model_file;
};

class RendererInterface {

public:
	virtual void render(Eigen::Matrix4f &pose_matrix, cv::Mat &depth, cv::Mat &color) = 0;
};

typedef RendererInterface* RendererInterfaceHandle;

class Renderer : public RendererInterface
{
public:
	Renderer(RendererConfiguration &configuration);
	void render(Eigen::Matrix4f &pose_matrix, cv::Mat &depth, cv::Mat &color);
private: 
	Eigen::Matrix3f intrinsics;

	Model model;
	Painter painter;
	
	float z_near;
	float z_far;
};

#ifdef _WIN32

#ifdef __cplusplus
#   define EXTERN_C     extern "C"
#else
#   define EXTERN_C
#endif // __cplusplus

#endif

#ifdef _WIN32
EXTERN_C RendererAPI
#endif
RendererInterfaceHandle get_depth_renderer(RendererConfiguration &configuration);

#endif
