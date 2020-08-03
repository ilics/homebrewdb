//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "refiner.h"
#include <util.h>
#include "rendering_helper.h"
#include "frame_payload.h"
#include "optimizer.h"
#include "correspondence_finder.h"
#include <string>
#include <Eigen/Geometry>
#include "occlusion_handler.hpp"
#include <filesystem>
#include <vis_utils.h>

using namespace Eigen;
using namespace std;

#define CAST_ROUND_INT(x) static_cast<int>(lround(x))

std::vector<Vector4f> extract_edges(const cv::Rect& roi, const cv::Mat& image)
{
	// Create and LSD detector with standard or no refinement.
	cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV, 0.9, 0.7);
	vector<cv::Vec4f> lines_std;

	// Detect the lines
	ls->detect(image(roi), lines_std);

	vector<Vector4f> edges(lines_std.size());

	transform(lines_std.begin(), lines_std.end(), edges.begin(), [&roi](const auto& line)
	{
		cv::Vec4f crop_line = line + cv::Vec4f(roi.x, roi.y, roi.x, roi.y);
		return Vector4f(crop_line[0], crop_line[1], crop_line[2], crop_line[3]);
	});

	return edges;
}

cv::Vec4i find_bounding_box_coordinates(const cv::Mat& img)
{
	int min_y = img.rows - 1;
	int min_x = img.cols - 1;
	int max_y = 0;
	int max_x = 0;

	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			float val = img.at<float>(i, j);

			if (val > 0.0)
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

	return cv::Vec4i(min_y, max_y, min_x, max_x);
}

void add_padding(cv::Vec4i& vec, int padding_pixels) {
	vec[0] = vec[0] - padding_pixels;
	vec[1] = vec[1] + padding_pixels;
	vec[2] = vec[2] - padding_pixels;
	vec[3] = vec[3] + padding_pixels;
}


template <typename T>
T get_median_absolute_deviation(const Matrix<T, Dynamic, 1>& v) {
	T median = get_median(v);
	Matrix<T, Dynamic, 1> res = (v - median).cwiseAbs();
	return get_median(res);
}

void compute_residual_std_dev(const vector<FramePayload>& frame_payloads, const Matrix4d& model_pose_m,
                              const Matrix3d& intrinsics, double& std_dev, bool& has_converged)
{
	vector<double> residuals;

	int converged_num = 0;
	const Isometry3d model_pose = to_isometry(model_pose_m);
	for (const auto& payload : frame_payloads)
	{
		const Correspondence& correspondence = payload.correspondence;
		Matrix4d scene_pose_inv = payload.scene_pose.inverse();

		Isometry3d world_to_cam_transform = to_isometry(scene_pose_inv) * model_pose;

		for (size_t i = 0; i < correspondence.get_number_of_correspondences(); i++)
		{
			const Vector3d world_point = correspondence.world_points[i].cast<double>();
			const Vector2d pixel_point = correspondence.corresponding_points[i].cast<double>();

			Vector3d point_in_cam_coords = world_to_cam_transform * world_point;
			point_in_cam_coords /= point_in_cam_coords[2];
			Vector2d pixel_coords = (intrinsics * point_in_cam_coords).block<2, 1>(0, 0);

			Vector2d rgb_edge_normal = correspondence.get_rgb_normal(i).cast<double>();
			double residual = (pixel_point - pixel_coords).dot(rgb_edge_normal);

			if (std::abs(residual) < 1.5)
			{
				converged_num++;
			}

			residuals.push_back(residual);
		}
	}

	VectorXd eigen_residuals = Eigen::Map<VectorXd, Unaligned>(residuals.data(), residuals.size());
	double mad = get_median_absolute_deviation(eigen_residuals);
	std_dev = 1.482579 * mad;

	auto res_num = static_cast<double>(residuals.size());
	has_converged = (converged_num / res_num) >= 0.9;
}

cv::Rect get_roi_box(const cv::Mat& depth_img, int model_padding_pixels)
{
	cv::Vec4i bbox = find_bounding_box_coordinates(depth_img);
	add_padding(bbox, model_padding_pixels);

	bbox[0] = max(0, bbox[0]);
	bbox[1] = min(depth_img.rows - 1, bbox[1]);

	bbox[2] = max(0, bbox[2]);
	bbox[3] = min(depth_img.cols - 1, bbox[3]);

	int crop_width = bbox[3] - bbox[2];
	int crop_height = bbox[1] - bbox[0];

	return cv::Rect(bbox[2], bbox[0], crop_width, crop_height);
}


Refiner::Refiner(const Configuration& p_configuration)
{
	configuration = p_configuration;
}

vector<Vector4f> get_depth_edges(const cv::Mat &depth_img, const cv::Rect &cropping_box)
{
	cv::Mat norm_depth_img;
	cv::normalize(depth_img, norm_depth_img, 255, 0, cv::NORM_MINMAX);
	norm_depth_img.convertTo(norm_depth_img, CV_8UC1);
	return extract_edges(cropping_box, norm_depth_img);
}

inline void draw_line_ids(cv::Mat &img, const vector<Vector4f> &edges) {
	for (int i = 0; i < edges.size(); i++) {
		const auto &edge = edges[i];
		cv::line(img, cv::Point(CAST_ROUND_INT(edge[0]), CAST_ROUND_INT(edge[1])),
			cv::Point(CAST_ROUND_INT(edge[2]), CAST_ROUND_INT(edge[3])), cv::Scalar(i), 1, 8);
	}
}

Matrix4d Refiner::refine_model_pose(const vector<Matrix4d>& camera_poses,
                                           const vector<cv::Mat>& grayscale_images,
                                           const Matrix4d& model_pose, int model_id)
{
	const auto& first_image = grayscale_images[0];

	int height = first_image.rows;
	int width = first_image.cols;

	CorrespondenceFinder correspondence_finder(configuration);
	RenderingHelper rendering_helper(configuration, model_id, width, height);

	const size_t number_of_frames = grayscale_images.size();
	Matrix4d current_model_pose = model_pose;

	int number_of_iterations = configuration.get_max_iterations();
	const Matrix3d intrinsics = configuration.get_intrinsics().cast<double>();

	for (int iteration = 0; iteration < number_of_iterations; iteration++)
	{
		vector<FramePayload> frame_payloads(number_of_frames);
		for (size_t frame_idx = 0; frame_idx < number_of_frames; ++frame_idx)
		{
			FramePayload payload;

			const Matrix4d& camera_pose = camera_poses[frame_idx];
			payload.scene_pose = camera_pose;

			Matrix4d world_to_camera = camera_pose.inverse() * current_model_pose;

			cv::Mat depth_img;
			cv::Mat rendered_color_img; // not used here
			rendering_helper.render(world_to_camera, depth_img, rendered_color_img);
			cv::Rect cropping_box = get_roi_box(depth_img, configuration.get_model_padding_pixels());
			
			vector<Vector4f> depth_edges = get_depth_edges(depth_img, cropping_box);
		
			const cv::Mat& grayscale_img = grayscale_images[frame_idx];
			cv::Mat edge_id_img = cv::Mat(height, width, CV_32SC1, cv::Scalar::all(-1));
			vector<Vector4f> rgb_edges = extract_edges(cropping_box, grayscale_img);

			draw_line_ids(edge_id_img, rgb_edges);
			transform(rgb_edges.begin(), rgb_edges.end(), back_inserter(payload.correspondence.rgb_edge_normals),
			          [](const Vector4f& edge) { return get_edge_normal(edge); });
			
			Matrix4d to_world_transformation = world_to_camera.inverse();
			correspondence_finder.find_correspondences(depth_img, edge_id_img, to_world_transformation,
			                                           depth_edges, payload.correspondence);
			

#ifdef _DEBUG
				if (frame_idx % 20 == 0)
				{
					cv::Mat edge_viz = get_edge_visualization(height, width, payload, depth_edges, rgb_edges);
					cv::imshow("Edge matches", edge_viz);
					cv::waitKey(0);
				}
			
#endif
			frame_payloads[frame_idx] = payload;
		}

		double res_std_dev;
		bool has_converged;
		compute_residual_std_dev(frame_payloads, current_model_pose, intrinsics, res_std_dev, has_converged);

		if (has_converged) break;

		current_model_pose = optimize_model_pose(frame_payloads, current_model_pose, intrinsics, res_std_dev);
	}

	return current_model_pose;
}


map<int, Matrix4d> Refiner::refine_model_poses(const RefinementInput& input)
{
	OcclusionHandler occlusion_handler(configuration, input.rgbd_image_files, input.camera_poses);

	map<int, Matrix4d> refined_model_poses;

	for (const auto& [model_id, model_pose] : input.model_poses)
	{
		vector<size_t> valid_frame_ids = occlusion_handler.get_frame_ids_with_visible_model(model_id, model_pose);
		if (!valid_frame_ids.empty())
		{
			cout << "Running optimization for model #" << model_id << " on " << valid_frame_ids.size() << " frames" << endl;

			vector<Matrix4d> camera_poses(valid_frame_ids.size());
			vector<cv::Mat> grayscale_images(valid_frame_ids.size());

			transform(valid_frame_ids.begin(), valid_frame_ids.end(), camera_poses.begin(),
			          [&input](const auto& frame_id) { return input.camera_poses[frame_id]; });
			transform(valid_frame_ids.begin(), valid_frame_ids.end(), grayscale_images.begin(),
			          [&input](const auto& frame_id)
			          {
				          const string& rgb_file = input.rgbd_image_files[frame_id].first;
				          cv::Mat rgb = read_image(rgb_file);
						  cv::Mat grayscale;
						  cv::cvtColor(rgb, grayscale, CV_RGB2GRAY);
						  return grayscale;
			          });


			refined_model_poses[model_id] = refine_model_pose(camera_poses, grayscale_images, model_pose, model_id);
		}
		else
		{
			refined_model_poses[model_id] = model_pose;
			cerr << "No valid frames for model #" << model_id << ", skipping refinement" << endl;
		}
	}

	return refined_model_poses;
}
