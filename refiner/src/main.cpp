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
#include "io_utils.hpp"
#include "configuration_parser.h"
#include <filesystem>
#include "iomanip"
#include <input_handler.h>

using namespace Eigen;
namespace fs = std::filesystem;
using namespace std;

#define CHECK_PARAM_EXISTS(parser, param_name) \
                if (!parser.has(param_name)) {\
                   cerr << "Missing parameter: " << param_name << endl;\
                   return EXIT_FAILURE;\
                }

#define CHECK_VALID_FILE(f) \
     if (!fs::is_regular_file(f)) {\
       cerr << "Invalid file: " << f << endl;\
       return EXIT_FAILURE;\
     }

#define CHECK_VALID_DIR(dir) \
      if (!fs::is_directory(dir)) {\
        cerr << "Invalid directory: " << dir << endl;\
        return EXIT_FAILURE;\
      }

void store_refined_poses(const map<int, Matrix4d>& object_poses, const string& scene_dir)
{
	string estimated_poses_dir = (fs::path(scene_dir)  / "refined_object_poses").string();

	if (!fs::is_directory(estimated_poses_dir))
	{
		fs::create_directories(estimated_poses_dir);
	}

	for (auto const& [model_id, pose] : object_poses)
	{
		stringstream filename_ss;
		filename_ss << std::setfill('0') << std::setw(6) << model_id << ".txt";
		const string filename = filename_ss.str();

		string out_file = (fs::path(estimated_poses_dir) / filename).string();

		ofstream out_filestream;
		out_filestream.open(out_file);
		out_filestream << pose << std::endl;
		out_filestream.close();
	}
}


int main(int argc, char* argv[])
{
	const string configuration_keys = "{help h usage ? |          | help on usage }"
		"{scene_dir   |          | path to the scene}"
		"{model_ids   |         -1 | Model id, default -1, i.e. refinement for all}"
		"{config_file   |          | path to config_json_file}";

	cv::CommandLineParser parser(argc, argv, configuration_keys);

	CHECK_PARAM_EXISTS(parser, "config_file")
	string config_file = parser.get<string>("config_file");
	CHECK_VALID_FILE(config_file)

	CHECK_PARAM_EXISTS(parser, "scene_dir")
	string scene_dir = parser.get<string>("scene_dir");
	CHECK_VALID_DIR(scene_dir)

	CHECK_PARAM_EXISTS(parser, "model_ids")
	string model_ids_string = parser.get<string>("model_ids");

	Configuration configuration;

	if (!parse_configuration(config_file, configuration))
	{
		cerr << "Failed to parse configuration, exiting" << endl;
		return EXIT_FAILURE;
	}

	RefinementInput refinement_input;

	if (!get_rgbd_files(scene_dir, refinement_input.rgbd_image_files)) return EXIT_FAILURE;
	if (!read_scene_camera_poses(scene_dir, refinement_input.camera_poses)) return EXIT_FAILURE;

	//number of frames must be equal to the number of poses
	if (refinement_input.rgbd_image_files.size() != refinement_input.camera_poses.size())
	{
		cerr << "The number of frames does not match the number of poses" << endl;
		return EXIT_FAILURE;
	}
	
	if (!get_model_poses(scene_dir, model_ids_string, refinement_input.model_poses)) return EXIT_FAILURE;

	Refiner refiner(configuration);
	const map<int, Matrix4d> refined_model_poses = refiner.refine_model_poses(refinement_input);
	store_refined_poses(refined_model_poses, scene_dir);

	return 0;
}
