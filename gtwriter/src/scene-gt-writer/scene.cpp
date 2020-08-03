//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "scene.h"
#include <cmath>
#include "frame.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
#define _USE_MATH_DEFINES

Scene::Scene(const Configuration &p_configuration, const std::string &p_scene_dir, 
		const std::vector<Model> &p_models, const std::vector<Eigen::Matrix4f> &p_camera_poses):
configuration(p_configuration), models(p_models), frame_poses(p_camera_poses), scene_dir(p_scene_dir)
{
	for (const auto& model : models)
	{
		scaled_renderers.emplace_back(ModelRenderer(model, configuration, true));
	}

}

size_t Scene::get_number_of_frames()
{
	return frame_poses.size();
}


inline int get_scaled_coordinate(int x, int cx, float focal_length_scale)
{
	return static_cast<int>(round(1.0f / focal_length_scale * (x - cx * (1.0f - focal_length_scale))));
}


Vector4i get_bounding_box(const cv::Mat& scaled_depth, const Matrix3f& scaled_intrinsics, float focal_length_scale)
{
	float cx = scaled_intrinsics(0, 2);
	float cy = scaled_intrinsics(1, 2);

	int min_y = scaled_depth.rows - 1;
	int min_x = scaled_depth.cols - 1;
	int max_y = 0;
	int max_x = 0;

	for (int i = 0; i < scaled_depth.rows; ++i)
	{
		for (int j = 0; j < scaled_depth.cols; ++j)
		{
			float val = scaled_depth.at<float>(i, j);

			if (isfinite(val) && val > 1e-3f)
			{
				if (i < min_y)
				{
					min_y = i;
				}
				else if (i > max_y)
				{
					max_y = i;
				}

				if (j < min_x)
				{
					min_x = j;
				}
				else if (j > max_x)
				{
					max_x = j;
				}
			}
		}
	}

	int min_y_scaled = get_scaled_coordinate(min_y, cy, focal_length_scale) - 1;
	int max_y_scaled = get_scaled_coordinate(max_y, cy, focal_length_scale) + 1;

	int min_x_scaled = get_scaled_coordinate(min_x, cx, focal_length_scale) - 1;
	int max_x_scaled = get_scaled_coordinate(max_x, cx, focal_length_scale) + 1;

	return { min_x_scaled, min_y_scaled, max_x_scaled - min_x_scaled, max_y_scaled - min_y_scaled };
}


vector<Frame> Scene::convert_to_scene_frames()
{
	size_t number_of_frames = frame_poses.size();
	size_t number_of_models = models.size();

	Matrix3f scaled_intrinsics = configuration.get_intrinsics();
	float focal_length_scale = configuration.get_focal_length_scale();

	scaled_intrinsics(0, 0) = focal_length_scale * scaled_intrinsics(0, 0);
	scaled_intrinsics(1, 1) = focal_length_scale * scaled_intrinsics(1, 1);

	vector<Frame> scene_frames(number_of_frames);

	for (int frame_idx = 0; frame_idx < number_of_frames; frame_idx++)
	{
		Frame frame;
		frame.frame_id = frame_idx;
		frame.scene_dir = scene_dir;

		cout << "Frame : " << frame_idx << endl;
		Matrix4f world_to_cam = frame_poses[frame_idx].inverse();

		for (int model_idx = 0; model_idx < number_of_models; model_idx++)
		{
			const Model& model = models[model_idx];

			ModelRenderer& scaled_renderer = scaled_renderers[model_idx];

			cv::Mat scaled_depth;
			cv::Mat scaled_color;

			Matrix4f model_pose = world_to_cam * model.canonical_pose;
			scaled_renderer.render(model_pose, scaled_depth, scaled_color);
			
			auto bbox = get_bounding_box(scaled_depth, scaled_intrinsics, focal_length_scale);
			frame.frame_models.emplace_back(FrameModel(model.model_id, model_pose, bbox));
		}

		scene_frames[frame_idx] = frame;
	}

	return scene_frames;
}
