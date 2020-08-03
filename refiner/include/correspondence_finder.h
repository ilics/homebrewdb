//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef CORRESPONDENCE_FINDER_H
#define CORRESPONDENCE_FINDER_H

#include <iostream>
#include <opencv/cv.hpp>
#include "configuration.h"
#include <Eigen/Core>

struct Correspondence
{
	std::vector<Eigen::Vector2f> corresponding_points;
	std::vector<Eigen::Vector3f> world_points;
	std::vector<Eigen::Vector4f> perpendicular_lines;

	std::vector<size_t> rgb_edges_ids;
	std::vector<Eigen::Vector2f> rgb_edge_normals;

	Eigen::Vector2f get_rgb_normal(size_t id) const {
	 size_t edge_id = rgb_edges_ids[id];
      return rgb_edge_normals[edge_id];
	}

	size_t get_number_of_correspondences() const
	{
		return corresponding_points.size();
	}
};

class CorrespondenceFinder
{
public:
	CorrespondenceFinder(const Configuration &configuration);
	void find_correspondences(const cv::Mat& depth_img, const cv::Mat& edge_id_img, const Eigen::Matrix4d &to_world_transformation_m, const std::vector<Eigen::Vector4f> &edges, Correspondence &correspondence);
private:
	bool match_closest(const cv::Mat &edge_id_img, const Eigen::Vector4f &perpendicular_direction, float pi, float pj, Correspondence &correspondence);

	Eigen::Matrix3d intrinsics;
	int edge_search_span;
	int point_sampling_step;
};

#endif