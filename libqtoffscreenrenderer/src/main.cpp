//######################################################################
//#   Liboffscreenrenderer Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include "renderer.h"

int main(int argc, char** argv)
{
	string model_file = argv[1];

	Eigen::Matrix3f rotation;
	Eigen::Vector3f translation;

	// rotation << -0.85453763, 0.51851374, 0.03013289,
	// 	0.25606695, 0.47106606, -0.84411363,
	// 	-0.45187887, -0.71361084, -0.53531733;
	//
	// translation << 0.00581056, -0.12364312, 0.97522034;
	rotation << 0.472679, -0.879703, -0.0519335,
		-0.530017, -0.236715, -0.814278,
		0.704029, 0.412418, -0.578148;

	translation << 0.0414942,
		0.0196108,
		0.566665;

	Eigen::Matrix3f intrinsics;
	// intrinsics << 1062.203, 0, 960.0,
	// 	0, 1060.9691, 540.0661,
	// 	0, 0, 1;

	intrinsics << 614.141479, 0, 421.837708,
		0, 613.991272, 248.147736,
		0, 0, 1;

	RendererConfiguration configuration;
	configuration.z_near = 0.001;
	configuration.z_far = 10.0;
	// configuration.height = 1080;
	// configuration.width = 1920;

	configuration.height = 480;
	configuration.width = 640;
	configuration.intrinsics = intrinsics;
	configuration.model_file = model_file;

	RendererInterface* renderer = get_depth_renderer(configuration);

	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose.block<3, 3>(0, 0) = rotation;
	pose.block<3, 1>(0, 3) = translation;

	cv::Mat depth, rgb;
    renderer->render(pose, depth, rgb);

	double minDepth = 0, maxDepth = 0;
	cv::minMaxLoc(depth, &minDepth, &maxDepth);
	cout << "min: " << minDepth << ", max: " << maxDepth << endl;

	cv::namedWindow("rendered-depth", cv::WINDOW_NORMAL);
	cv::imshow("rendered-depth", depth);
	cv::resizeWindow("rendered-depth", 640, 480);
	cv::waitKey(0);

	delete renderer;
	return 0;
}