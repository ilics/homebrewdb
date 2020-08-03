//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <opencv2/highgui/highgui.hpp>
#include "visualizer.hpp"
#include "pose_validator.hpp"

using namespace cv;
using namespace Eigen;
using namespace std;

float get_median(const vector<float> &detections) {

	size_t size = detections.size();
	if (size < 3) {
		return -1;
	}

	vector<float> detections_copy = detections;
	sort(detections_copy.begin(), detections_copy.end());

	if (size % 2 == 0) {
		return (detections_copy[size / 2 - 1] + detections_copy[size / 2]) / 2.0f;
	}

	return detections_copy[size / 2];
}

vector<vector<Vector3f>> sample_points_along_edges(const vector<Vector3f>& points_3d)
{
	vector<vector<Vector3f>> edge_samples;
	edge_samples.reserve(4);

	for (int i = 0; i < 4; i++) {
		edge_samples.emplace_back();
	}

	float board_edge_length = points_3d[1][1] - points_3d[0][1];

	float step = board_edge_length / 15.0f;

	for (float cur_point = step; cur_point <= board_edge_length; cur_point += step) {

		Vector3f e0_sample = { points_3d[0][0], points_3d[0][1] + cur_point, points_3d[0][2] };
		edge_samples[0].push_back(e0_sample);

		Vector3f e1_sample = { points_3d[1][0] + cur_point, points_3d[1][1], points_3d[1][2] };
		edge_samples[1].push_back(e1_sample);

		Vector3f e2_sample = { points_3d[2][0], points_3d[2][1] - cur_point, points_3d[2][2] };
		edge_samples[2].push_back(e2_sample);

		Vector3f e3_sample = { points_3d[3][0] - cur_point, points_3d[3][1], points_3d[3][2] };
		edge_samples[3].push_back(e3_sample);
	}

	return edge_samples;
}

bool verify_pose_correctness(const vector<float> &volume , const Isometry3f &pose_inverse, const Matrix3f &intrinsics,
	const Mat &depth_image, const Mat &image_rgb, float error_threshold) {
	const vector<Vector3f> selected_3d_points = { {volume[0], volume[2], volume[5]},
											 {volume[0], volume[3], volume[5]},
											 {volume[1], volume[3], volume[5]},
											 {volume[1], volume[2], volume[5]}
	};

	Mat image_to_draw = image_rgb.clone();

	vector<vector<Vector3f>> edge_samples = sample_points_along_edges(selected_3d_points);
	vector<vector<Vector2f>> projected_edge_samples;

	for (const auto &samples : edge_samples) {

		vector<Vector2f> projected_samples;

		for (const auto &point : samples) {

			Vector3f point_3d = pose_inverse * point;
			point_3d = point_3d / point_3d[2];
			point_3d = intrinsics * point_3d;
			Vector2f point_2d = { point_3d[0], point_3d[1] };

			projected_samples.push_back(point_2d);
		}

		projected_edge_samples.push_back(projected_samples);
	}

	vector<float> medians;
	const Matrix3f intrinsics_inverse = intrinsics.inverse();
	const Isometry3f camera_to_world = pose_inverse.inverse();

	vector<Vector3f> estimated_world_points;

	for (int i = 0; i < 4; i++) {

		vector<Vector2f> &current_projected_edge_samples = projected_edge_samples[i];
		vector<Vector3f> &gt_edge_samples_3d = edge_samples[i];

		vector<float> edge_distances;

		for (int j = 0; j < gt_edge_samples_3d.size(); j++) {
			Vector2f current_projected_sample = current_projected_edge_samples[j];

			float x_f = current_projected_sample[0];
			float y_f = current_projected_sample[1];

			int x = lroundf(x_f);
			int y = lroundf(y_f);

			if (x >= 0 && y >= 0 && x < depth_image.cols && y < depth_image.rows) {
				float depth = depth_image.at<float>(y, x);

				if (isfinite(depth) && depth > 0.0) {
					Vector3f hom_point = { x_f, y_f, 1.0f };
					Vector3f unprojected = depth * intrinsics_inverse * hom_point;

					Vector3f estimated_sample_world_coordinate = camera_to_world * unprojected;

					estimated_world_points.push_back(estimated_sample_world_coordinate);

					Vector3f gt_sampled_world_point = gt_edge_samples_3d[j];

					float distance = (estimated_sample_world_coordinate - gt_sampled_world_point).norm();
					edge_distances.push_back(distance);
				}
			}
		}

		medians.push_back(get_median(edge_distances));
	}

	int edge_count_with_matches = 0;

	for (auto &median : medians) {
		if (median >= 0 && median < error_threshold) {
			edge_count_with_matches++;
		}
	}

	return edge_count_with_matches > 2;
}

bool validate_pose(const PoseValidationPayload &payload)
{
	Isometry3f pose_inverse = payload.pose.inverse();

	const float validation_treshold_meters = 0.009;

	bool is_correct = verify_pose_correctness(payload.volume, pose_inverse, payload.intrinsics,
		payload.depth_image, payload.rgb_image, validation_treshold_meters);

	if (!is_correct)
	{
		visualize_volume(payload.rgb_image, payload.intrinsics, pose_inverse, payload.volume);
		int key = cv::waitKey(0);
		is_correct = key != 's' && key != 'S';
	}

	return is_correct;
}
