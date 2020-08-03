//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "occlusion_handler.hpp"

#include <utility>
#include "rendering_helper.h"

using namespace Eigen;
using namespace std;


OcclusionHandler::OcclusionHandler(Configuration configuration,
                                   vector<RgbdFile> rgbd_image_files,
                                   vector<Matrix4d> camera_poses) :
	configuration(move(configuration)), rgbd_image_files(move(rgbd_image_files)), camera_poses(move(camera_poses))
{
	object_visibility_threshold = configuration.get_object_visibility_threshold();
	occlusion_threshold = configuration.get_occlusion_threshold();
}


bool OcclusionHandler::is_object_visible(const cv::Mat& gt_depth, const cv::Mat& rendered_depth)
{
	int total_object_pixels = 0;
	int visible_object_pixels = 0;

	for (int i = 0; i < gt_depth.rows; ++i)
	{
		for (int j = 0; j < gt_depth.cols; ++j)
		{
			float rendered_val = rendered_depth.at<float>(i, j);

			if (isfinite(rendered_val) && rendered_val > 1e-3f)
			{
				float gt_val = gt_depth.at<float>(i, j);

				if (isfinite(gt_val) && gt_val > 1e-3f)
				{
					float diff = rendered_val - gt_val;
					
					// probably no occlusion
					if (diff < occlusion_threshold)
					{
						visible_object_pixels++;
					}
				} else
				{
					visible_object_pixels++;
				}

				total_object_pixels++;
			}
		}
	}

	const float visible_fraction = static_cast<float>(visible_object_pixels) / static_cast<float>(total_object_pixels);

	return visible_fraction > object_visibility_threshold;
}


vector<size_t> OcclusionHandler::get_frame_ids_with_visible_model(int model_id, const Matrix4d& model_pose)
{
	size_t number_of_frames = rgbd_image_files.size();

	const string& depth_file = rgbd_image_files[0].second;
	cv::Mat first_depth_image = cv::imread(depth_file, -1);

	int width = first_depth_image.cols;
	int height = first_depth_image.rows;

	RenderingHelper rendering_helper(configuration, model_id, width, height);

	vector<size_t> indices;

	for (size_t i = 0; i < number_of_frames; ++i)
	{
		const cv::Mat depth_img = cv::imread(rgbd_image_files[i].second, -1);
		cv::Mat gt_depth_float;
		depth_img.convertTo(gt_depth_float, CV_32FC1, 0.001);

		cv::Mat rendered_depth;
		cv::Mat rendered_rgb;

		Matrix4d world_to_cam = camera_poses[i].inverse() * model_pose;
		rendering_helper.render(world_to_cam, rendered_depth, rendered_rgb);

		if (is_object_visible(gt_depth_float, rendered_depth))
		{
			indices.push_back(i);
		} else
		{
			cout << "Model #" << model_id << " is invisible in frame #" << i << ", skipping" << endl;
		}
	}

	return indices;
}
