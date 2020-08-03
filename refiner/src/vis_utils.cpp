//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "vis_utils.h"
#include <filesystem>

namespace fs = std::filesystem;
using namespace std;
#define CAST_ROUND_INT(x) static_cast<int>(lround(x))

inline void drawLines(cv::Mat &img, const std::vector<Eigen::Vector4f> &edges, const cv::Scalar &color) {
	for (const auto &line_seg : edges) {
		cv::line(img, cv::Point(CAST_ROUND_INT(line_seg[0]), CAST_ROUND_INT(line_seg[1])),
			cv::Point(CAST_ROUND_INT(line_seg[2]), CAST_ROUND_INT(line_seg[3])), color, 1, 8);

	}
}


inline void drawCircles(cv::Mat &img, const std::vector<Eigen::Vector2f> &edges, cv::Scalar color) {

	for (const auto &line_seg : edges) {
		cv::circle(img, cv::Point((int)line_seg[0], (int)line_seg[1]), 2, color, 1, 8);
	}
}

cv::Mat get_edge_visualization(int height, int width,
	const FramePayload& payload, const vector<Eigen::Vector4f>& depth_edges, const vector<Eigen::Vector4f>& rgb_edges)
{
	cv::Mat edge_img(height, width, CV_8UC3, cv::Scalar::all(0));
	cv::Scalar color_green = cv::Scalar(1, 255, 0);
	cv::Scalar y = cv::Scalar(1, 255, 255);

	cv::Scalar color_red = cv::Scalar(255, 1, 1);
	drawLines(edge_img, depth_edges, color_green);
	drawLines(edge_img, payload.correspondence.perpendicular_lines, color_green);
	drawLines(edge_img, rgb_edges, color_red);
	drawCircles(edge_img, payload.correspondence.corresponding_points, y);

	return edge_img;
}

void save_edge_visualization(int height, int width, const string& edge_match_out_path, int num_iter,
	const FramePayload& payload, const vector<Eigen::Vector4f>& depth_edges, const vector<Eigen::Vector4f>& rgb_edges,
	const string& in_img_filename)
{
	cv::Mat edge_img(height, width, CV_8UC3, cv::Scalar::all(0));
	cv::Scalar color_green = cv::Scalar(1, 255, 0);
	cv::Scalar y = cv::Scalar(1, 255, 255);

	cv::Scalar color_red = cv::Scalar(255, 1, 1);
	drawLines(edge_img, depth_edges, color_green);
	drawLines(edge_img, payload.correspondence.perpendicular_lines, color_green);
	drawLines(edge_img, rgb_edges, color_red);
	drawCircles(edge_img, payload.correspondence.corresponding_points, y);

	size_t lastindex = in_img_filename.find_last_of(".");
	std::string raw_in_file_name = in_img_filename.substr(0, lastindex);

	std::stringstream ss_out_dir;
	ss_out_dir << edge_match_out_path << "/" << raw_in_file_name;
	std::string out_dir = ss_out_dir.str();

	if (!fs::is_directory(out_dir))
	{
		fs::create_directories(out_dir);
	}

	std::stringstream edge_match_file_ss;
	edge_match_file_ss << out_dir << "/it_" << std::setfill('0') << std::setw(3) << num_iter << "_" << in_img_filename;
	std::string edge_match_file = edge_match_file_ss.str();

	cv::imwrite(edge_match_file, edge_img);
}


