//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "output_writer.h"
#include "configuration.h"
#include <filesystem>
#include <nlohmann/json.hpp>
#include <fstream>
#include "util.h"

namespace fs = std::filesystem;
using namespace std;
using namespace Eigen;

using json = nlohmann::json;

inline void make_dirs_if_not_exists(const string& dir)
{
	if (!fs::is_directory(dir))
	{
		fs::create_directories(dir);
	}
}

vector<float> get_rotation_vector(const Matrix4f& m)
{
	vector<float> rot_vector(9);
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			int idx = i * 3 + j;
			rot_vector[idx] = m(i, j);
		}
	}

	return rot_vector;
}

inline vector<float> get_translation_vector(const Matrix4f& m)
{
	return { m(0, 3), m(1, 3), m(2, 3) };
}

vector<float> get_intrinsics_vector(const Matrix3f &intrinsics)
{
	vector<float> intrinsics_vector(9);
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			int idx = i * 3 + j;
			intrinsics_vector[idx] = intrinsics(i, j);
		}
	}

	return intrinsics_vector;
}

json get_frame_entry(const Frame &frame)
{
	json frame_entry;
	for (const auto &frame_model : frame.frame_models)
	{
		json object_entry;
		object_entry["cam_R_m2c"] = get_rotation_vector(frame_model.pose);
		object_entry["cam_t_m2c"] = get_translation_vector(frame_model.pose);
		vector<int> bbox = { frame_model.bbox(0), frame_model.bbox(1), frame_model.bbox(2), frame_model.bbox(3) };
		object_entry["obj_bb"] = bbox;
		object_entry["obj_id"] = to_string(frame_model.model_id);

		frame_entry.push_back(object_entry);
	}

	return frame_entry;
}

void write_scene_gt_labels(const string &gt_scenes_file, const std::vector<Frame> &joined_sequences)
{
	json out_json;

	for (size_t i = 0; i < joined_sequences.size(); ++i)
	{
		auto idx_string = to_string(i);
		out_json[idx_string] = get_frame_entry(joined_sequences[i]);
		
	}

	std::ofstream ofs(gt_scenes_file);
	ofs << std::setw(4) << out_json << std::endl;
}


inline json create_camera_entry(const Matrix3f &intrinsics)
{
	json camera_entry;
	camera_entry["cam_K"] = get_intrinsics_vector(intrinsics);
	return camera_entry;
}


void write_gt_camera_labels(const Configuration &configuration, const string &gt_camera_file, size_t num_frames)
{
	json out_json;

	json camera_entry = create_camera_entry(configuration.get_intrinsics());

	for (size_t i = 0; i < num_frames; ++i)
	{
		out_json[to_string(i)] = camera_entry;
	}

	std::ofstream ofs(gt_camera_file);
	ofs << std::setw(4) << out_json << std::endl;
}

void copy_images(const Configuration &configuration, const std::vector<Frame> &joined_sequences)
{
	string out_dir = configuration.get_out_dir();
	string out_rgb_dir = (fs::path(out_dir) / "rgb").string();
	string out_depth_dir = (fs::path(out_dir) / "depth").string();

	make_dirs_if_not_exists(out_rgb_dir);
	make_dirs_if_not_exists(out_depth_dir);

	for (size_t i = 0; i < joined_sequences.size(); ++i)
	{
		const Frame &frame = joined_sequences[i];
		string images_dir = (fs::path(frame.scene_dir) / "images").string();

		string rgb_dir = (fs::path(images_dir) / "rgb").string();
		string depth_dir = (fs::path(images_dir) / "depth").string();

		string current_filename = get_image_filename_for_idx(frame.frame_id);
		string current_rgb_file = (fs::path(rgb_dir) / current_filename).string();
		string current_depth_file = (fs::path(depth_dir) / current_filename).string();

		string out_filename = get_image_filename_for_idx(i);
		string out_rgb_file = (fs::path(out_rgb_dir) / out_filename).string();
		string out_depth_file = (fs::path(out_depth_dir) / out_filename).string();

		fs::copy_file(current_rgb_file, out_rgb_file, fs::copy_options::overwrite_existing);
		fs::copy_file(current_depth_file, out_depth_file, fs::copy_options::overwrite_existing);
	}
}

void write_output(const Configuration &configuration, const std::vector<Frame> &joined_sequences)
{
	string out_dir = configuration.get_out_dir();
	make_dirs_if_not_exists(out_dir);

	string gt_scene_file = (fs::path(out_dir) / "scene_gt.json").string();
	string gt_camera_file = (fs::path(out_dir) / "camera_gt.json").string();

	write_scene_gt_labels(gt_scene_file, joined_sequences);
	write_gt_camera_labels(configuration, gt_camera_file, joined_sequences.size());

	if (configuration.get_copy_images())
	{
		copy_images(configuration, joined_sequences);
	}
}
