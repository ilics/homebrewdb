//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef IOUTILS_HPP
#define IOUTILS_HPP

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rgbd_types.hpp"
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace fs = std::filesystem;


inline RgbdFrame get_rgbd_frame(const RgbdFile &rgbd_file)
{
	cv::Mat rgb = cv::imread(rgbd_file.first);
	cv::Mat depth = cv::imread(rgbd_file.second, -1);

	cv::Mat depth_float;
	depth.convertTo(depth_float, CV_32FC1, 0.001);

	return RgbdFrame(rgb, depth_float);
}


template<typename T>
inline std::vector<Eigen::Matrix<T, 4, 4>> read_scene_poses(const std::string &poses_file)
{
	std::ifstream infile(poses_file);
	std::string line;

	std::vector<Eigen::Matrix<T, 4, 4>> poses;
	int i = 0;

	Eigen::Matrix<T, 4, 4> temp_matrix;

	while (std::getline(infile, line))
	{
		if (i % 5 == 0)
		{
			if (i > 0)
			{
				poses.push_back(temp_matrix);
				temp_matrix = Eigen::Matrix<T, 4, 4>();
			}
		}
		else
		{
			int row = (i % 5) - 1;
			std::stringstream ss(line);

			T current_value;
			int col = 0;

			while (ss >> current_value)
			{
				temp_matrix(row, col) = current_value;
				col++;
			}
		}
		i++;
	}

	poses.push_back(temp_matrix);

	return poses;
}

template <typename T>
inline void store_poses(const std::vector<Eigen::Transform<T, 3, Eigen::Isometry>> &subsampled_valid_poses, const std::string &out_poses_file)
{
	std::ofstream poses_out_stream;

	poses_out_stream.open(out_poses_file);
	for (int i = 0; i < static_cast<int>(subsampled_valid_poses.size()); ++i)
	{
		const Eigen::Matrix<T, 4, 4> &out_pose = subsampled_valid_poses[i].matrix();
		poses_out_stream << i << std::endl << out_pose << std::endl;
	}

	poses_out_stream.close();
}


inline void store_images(const std::vector<RgbdFile> &rgbd_files, const std::string &out_rgb_dir, const std::string &out_depth_dir)
{
	int i = 0;
	for (const auto &rgbd_file : rgbd_files)
	{
		const std::string &in_rgb_file = rgbd_file.first;
		const std::string &in_depth_file = rgbd_file.second;

		std::stringstream filename_ss;
		filename_ss << std::setfill('0') << std::setw(6) << i << ".png";
		const std::string filename = filename_ss.str();

		std::string out_rgb_file = (fs::path(out_rgb_dir) / filename).string();
		std::string out_depth_file = (fs::path(out_depth_dir) / filename).string();

		fs::copy_file(in_rgb_file, out_rgb_file, fs::copy_options::overwrite_existing);
		fs::copy_file(in_depth_file, out_depth_file, fs::copy_options::overwrite_existing);

		i++;
	}
}

inline std::string to_plain_filename(const std::string &f)
{
	return fs::path(f).filename().string();
}

inline std::vector<RgbdFile> get_input_images(const std::string &rgb_images_dir, const std::string &depth_images_dir)
{
	cv::String rgb_file_pattern = (fs::path(rgb_images_dir) / "*.png").string();
	cv::String depth_file_pattern = (fs::path(depth_images_dir) / "*.png").string();

	std::vector<cv::String> rgb_files;
	std::vector<cv::String> depth_files;

	cv::glob(rgb_file_pattern, rgb_files);
	cv::glob(depth_file_pattern, depth_files);

	std::vector<std::string> rgb_filenames(rgb_files.size());
	std::vector<std::string> depth_filenames(depth_files.size());

	std::transform(rgb_files.begin(), rgb_files.end(), rgb_filenames.begin(), to_plain_filename);
	std::transform(depth_files.begin(), depth_files.end(), depth_filenames.begin(), to_plain_filename);

	std::sort(rgb_filenames.begin(), rgb_filenames.end());
	std::sort(depth_filenames.begin(), depth_filenames.end());

	std::vector<std::string> image_filenames;
	std::set_intersection(rgb_filenames.begin(), rgb_filenames.end(),
		depth_filenames.begin(), depth_filenames.end(), back_inserter(image_filenames));

	std::vector<RgbdFile> rgbd_files(image_filenames.size());

	std::transform(image_filenames.begin(), image_filenames.end(), rgbd_files.begin(), [&rgb_images_dir, &depth_images_dir](const std::string &fn)
	{
		const std::string &rgb_file = (fs::path(rgb_images_dir) / fn).string();
		const std::string &depth_file = (fs::path(depth_images_dir) / fn).string();
		return RgbdFile(rgb_file, depth_file);
	});

	return rgbd_files;
}


#endif