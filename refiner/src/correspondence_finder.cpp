//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "correspondence_finder.h"
#include <util.h>
#include <cmath>

using namespace Eigen;
using namespace std;


#define CAST_ROUND_INT(x) static_cast<int>(lround(x))

CorrespondenceFinder::CorrespondenceFinder(const Configuration& configuration)
{
	intrinsics = configuration.get_intrinsics().cast<double>();
	edge_search_span = configuration.get_edge_search_span();
	point_sampling_step = configuration.get_point_sampling_step();
}

Vector4f get_perpendicular_direction(const Vector4f& line_endpoints)
{
	float v_x = line_endpoints[0] - line_endpoints[2];
	float v_y = line_endpoints[1] - line_endpoints[3];

	float magnitude = sqrt(v_x * v_x + v_y * v_y);
	float nv_x = v_x / magnitude;
	float nv_y = v_y / magnitude;

	// 90 degree rotation
	float temp = nv_x;
	nv_x = -nv_y;
	nv_y = temp;

	return Vector4f(nv_x, nv_y, v_x, v_y);
}


template <typename T>
T get_edge_id_in_the_neighborhood(float& x, float& y, cv::Mat edge_img)
{
	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			int y_off = CAST_ROUND_INT(y) + i;
			int x_off = CAST_ROUND_INT(x) + j;

			if (y_off < edge_img.rows && x_off < edge_img.cols && y_off >= 0 && x_off >= 0)
			{
				T val = edge_img.at<T>(y_off, x_off);

				if (val != -1)
				{
					y = y + static_cast<float>(i);
					x = x + static_cast<float>(j);
					return val;
				}
			}
		}
	}

	return T(-1);
}

inline float get_line_length(const Vector4f& v) {
	return sqrt(powf(v[0] - v[2], 2) + powf(v[1] - v[3], 2));
}

float get_depth_in_the_neighborhood(const cv::Mat& depth_img, float x, float y)
{
	int num_valid_values = 0;
	float depth_sum = 0.0;

	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			int y_off = CAST_ROUND_INT(y) + i;
			int x_off = CAST_ROUND_INT(x) + j;

			if (y_off < depth_img.rows && x_off < depth_img.cols && y_off >= 0 && x_off >= 0)
			{
				float val = depth_img.at<float>(y_off, x_off);
				if (isfinite(val) && val > .0f)
				{
					depth_sum += val;
					num_valid_values++;
				}
			}
		}
	}

	if (num_valid_values > 0)
	{
		return depth_sum / static_cast<float>(num_valid_values);
	}

	return .0f;
}

bool CorrespondenceFinder::match_closest(const cv::Mat& edge_id_img, const Vector4f& perpendicular_direction, float pi, float pj,
                                          Correspondence& correspondence) {
	for (int i = 1; i < edge_search_span; ++i)
	{
		float c_x = pi + perpendicular_direction[0] * static_cast<float>(i);
		float c_y = pj + perpendicular_direction[1] * static_cast<float>(i);

		float d_x = pi - perpendicular_direction[0] * static_cast<float>(i);
		float d_y = pj - perpendicular_direction[1] * static_cast<float>(i);

		int val = get_edge_id_in_the_neighborhood<int>(c_x, c_y, edge_id_img);
		
		if (val != -1)
		{
			correspondence.corresponding_points.push_back({c_x, c_y});
			correspondence.rgb_edges_ids.push_back(val);
			return true;
		}

		val = get_edge_id_in_the_neighborhood<int>(d_x, d_y, edge_id_img);
		
		if (val != -1)
		{
			correspondence.corresponding_points.push_back({d_x, d_y});
			correspondence.rgb_edges_ids.push_back(val);
			return true;
		}
	}

	return false;
}

void CorrespondenceFinder::find_correspondences(const cv::Mat& depth_img, const cv::Mat& edge_id_img,
                                                const Matrix4d& to_world_transformation_m,
                                                const vector<Vector4f>& edges, Correspondence& correspondence)
{

	Isometry3d to_world_transformation = to_isometry(to_world_transformation_m);

	for (auto& line_endpoints : edges) {
		Vector4f perpendicular_direction = get_perpendicular_direction(line_endpoints);
		float line_length = get_line_length(line_endpoints);

		int num_of_edge_sample_points = static_cast<int>(line_length / static_cast<float>(point_sampling_step));
		float sampling_step = 1.f / static_cast<float>(num_of_edge_sample_points);

		for (int n = 1; n < num_of_edge_sample_points; ++n) {
			float pi = line_endpoints[0] - perpendicular_direction[2] * (static_cast<float>(n) * sampling_step);
			float pj = line_endpoints[1] - perpendicular_direction[3] * (static_cast<float>(n) * sampling_step);

			float depth = get_depth_in_the_neighborhood(depth_img, pi, pj);

			if (!isfinite(depth) || abs(depth) < 1e-5) continue;

			double xp = (pi - intrinsics(0, 2)) * depth / intrinsics(0, 0);
			double yp = (pj - intrinsics(1, 2)) * depth / intrinsics(1, 1);

			Vector3d world_point = to_world_transformation * Vector3d(xp, yp, depth);

			float c_x = pi + perpendicular_direction[0] * static_cast<float>(edge_search_span);
			float c_y = pj + perpendicular_direction[1] * static_cast<float>(edge_search_span);

			float d_x = pi - perpendicular_direction[0] * static_cast<float>(edge_search_span);
			float d_y = pj - perpendicular_direction[1] * static_cast<float>(edge_search_span);

			correspondence.perpendicular_lines.push_back({c_x, c_y, d_x, d_y});

			if (match_closest(edge_id_img, perpendicular_direction, pi, pj, correspondence))
			{
				correspondence.world_points.push_back(world_point.cast<float>());
			}
		}
	}
}
