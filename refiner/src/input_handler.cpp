//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "input_handler.h"
#include <util.h>
#include "io_utils.hpp"
#include <filesystem>
#include "iomanip"
#include <iostream>
#include <map>

using namespace Eigen;
namespace fs = std::filesystem;
using namespace std;


#define CHECK_VALID_DIR(dir) \
     if (!fs::is_directory(dir)) {\
       cerr << "Invalid directory: " << dir << endl;\
       return false;\
     }

#define CHECK_VALID_FILE(f) \
     if (!fs::is_regular_file(f)) {\
       cerr << "Invalid file: " << f << endl;\
       return false;\
     }


bool read_scene_camera_poses(const string& scene_dir, vector<Eigen::Matrix4d>& camera_poses)
{
	const string camera_poses_file = (fs::path(scene_dir) / "camera_poses.txt").string();
	CHECK_VALID_FILE(camera_poses_file)

	camera_poses = read_scene_poses<double>(camera_poses_file);
	return true;
}

bool get_rgbd_files(const string& scene_dir, vector<RgbdFile>& rgbd_files)
{
	const fs::path images_path = fs::path(scene_dir) / "images";

	const string rgb_dir = (images_path / "rgb").string();
	const string depth_dir = (images_path / "depth").string();

	CHECK_VALID_DIR(rgb_dir);
	CHECK_VALID_DIR(depth_dir);

	rgbd_files = get_input_images(rgb_dir, depth_dir);
	return true;
}


bool parse_model_ids(const string& model_ids_string, vector<int>& model_ids)
{
	stringstream ids_ss(model_ids_string);
	try
	{
		for (int i; ids_ss >> i;)
		{
			model_ids.push_back(i);
			if (ids_ss.peek() == ',')
				ids_ss.ignore();
		}
	}
	catch (const std::exception&)
	{
		cerr << "Invalid model ids: " << model_ids_string << endl;
		return false;
	}

	return true;
}

void get_all_model_pose_files(const string& object_poses_dir, map<int, string>& model_pose_files)
{
	for (const auto& entry : fs::directory_iterator(object_poses_dir))
	{
		const fs::path& file_path = entry.path();
		const string extension = file_path.extension().string();

		if (extension == ".txt")
		{
			string filename = file_path.filename().string();
			string raw_name = filename.substr(0, filename.find_last_of("."));

			int model_id = std::stoi(raw_name);
			model_pose_files[model_id] = file_path.string();
		}
	}
}

void get_selected_model_pose_files(const string& object_poses_dir, const vector<int>& model_ids,
                                   map<int, string>& model_pose_files)
{
	for (const auto model_id : model_ids)
	{
		std::stringstream filename_ss;
		filename_ss << std::setfill('0') << std::setw(6) << model_id << ".txt";
		const std::string filename = filename_ss.str();

		const string model_file = (fs::path(object_poses_dir) / filename).string();
		if (fs::is_regular_file(model_file))
		{
			cout << "Adding pose file for model #" << model_id << endl;
			model_pose_files[model_id] = model_file;
		}
		else
		{
			cerr << "Model file does not exists for id #" << model_id << ", skipping" << endl;
		}
	}
}

map<int, string> get_model_poses_files(const string& scene_dir, const vector<int>& model_ids)
{
	map<int, string> model_pose_files;

	const string object_poses_dir = (fs::path(scene_dir) / "object_poses").string();
	// extract all poses
	if (model_ids.size() == 1 && model_ids[0] == -1)
	{
		get_all_model_pose_files(object_poses_dir, model_pose_files);
	}
	else
	{
		get_selected_model_pose_files(object_poses_dir, model_ids, model_pose_files);
	}

	return model_pose_files;
}

bool get_model_poses(const string& scene_dir, const string& model_ids_string, map<int, Eigen::Matrix4d>& model_poses)
{
	vector<int> model_ids;
	if (!parse_model_ids(model_ids_string, model_ids)) return false;

	map<int, string> model_poses_files = get_model_poses_files(scene_dir, model_ids);

	for (auto const& [model_id, pose_file] : model_poses_files)
	{
		model_poses[model_id] = read_pose_from_file<double>(pose_file);
	}

	return true;
}
