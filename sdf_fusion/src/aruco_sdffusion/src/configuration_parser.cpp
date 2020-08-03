//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "configuration_parser.hpp"
#include <opencv2/core.hpp>
#include <filesystem>
#include "utils.hpp"

using namespace std;
namespace fs = std::filesystem;

const string configuration_keys =
"{help h usage ? |          | help on usage}"
"{images_dir   |          | input images dir containing /rgb and /depth dirs}"
"{config_dir   |          | config dir containing board.yml and dict.yml}"
"{intrinsics_file |          | file containing row-wise intrinsic matrix}"
"{marker_size  |    0.0491    | marker size in meters}"
"{voxel_size  |   0.002       | voxel size}"
"{preprocess  |          |   do pose estimation and discarding of frames }"
"{req_num_frames  |          | }"
"{out_dir |          | out directory for storing camera poses, filtered images, reconstructed mesh}";

#define CHECK_PARAM_EXISTS(parser, param_name) \
             if (!parser.has(param_name)) {\
			    cerr << "Missing parameter: " << param_name << endl;\
				return false;\
			 }

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

#define CHECK_VALID_NUM_FRAMES(n) \
	if (n <= 0) {\
	  cerr << "Invalid number of frames: " << n << endl;\
	  return false;\
    }

void create_if_not_exists(const string &dir)
{
	if (!fs::is_directory(dir))
	{
		fs::create_directories(dir);
	}
}

void resolve_out_dir(string out_dir, Configuration &configuration)
{
	create_if_not_exists(out_dir);

	string out_images_dir = (fs::path(out_dir) / "images").string();

	string out_rgb_images_dir = (fs::path(out_images_dir) / "rgb").string();
	create_if_not_exists(out_rgb_images_dir);

	string out_depth_images_dir = (fs::path(out_images_dir) / "depth").string();
	create_if_not_exists(out_depth_images_dir);

	configuration.set_out_dir(out_dir);
	configuration.set_out_rgb_images_dir(out_rgb_images_dir);
	configuration.set_out_depth_images_dir(out_depth_images_dir);
}

void resolve_intrinsics(const string &intrinsics_file, Configuration& configuration)
{
	Eigen::Matrix3f intrinsics = read_intrinsics_from_file<float>(intrinsics_file);
	configuration.set_intrinsics(intrinsics);
}

bool resolve_configuration(int argc, char** argv, Configuration &configuration)
{
	cv::CommandLineParser parser(argc, argv, configuration_keys);
	
	if (parser.has("help")) {
		parser.printMessage();
		cout << "Example use:" << endl;
		cout << "aruco_sdffusion -images_dir=</path/to/images> -config_dir=</path/to/config> "
		"-voxel_size=<voxel_size> -marker_size=<marker size>"
		"-intrinsics_file=</path/to/intrinsics.txt> -out_dir=</path/to/out_dir> -preprocess" << endl;
		return false;
	}

	CHECK_PARAM_EXISTS(parser, "images_dir");
	string images_dir = parser.get<string>("images_dir");
	CHECK_VALID_DIR(images_dir)

	string rgb_images_dir = (fs::path(images_dir) / "rgb").string();
	CHECK_VALID_DIR(rgb_images_dir)
	string depth_images_dir = (fs::path(images_dir) / "depth").string();
	CHECK_VALID_DIR(depth_images_dir)

	CHECK_PARAM_EXISTS(parser, "config_dir");
	string config_dir = parser.get<string>("config_dir");
	CHECK_VALID_DIR(config_dir)

	CHECK_PARAM_EXISTS(parser, "intrinsics_file");
	string intrinsics_file = parser.get<string>("intrinsics_file");
	CHECK_VALID_FILE(intrinsics_file);

	CHECK_PARAM_EXISTS(parser, "out_dir");
	string out_dir = parser.get<string>("out_dir");

	CHECK_PARAM_EXISTS(parser, "marker_size");
	float marker_size = parser.get<float>("marker_size");

	CHECK_PARAM_EXISTS(parser, "voxel_size");
	float voxel_size = parser.get<float>("voxel_size");

	string dictionary_file = (fs::path(config_dir) / "dict.yml").string();
	CHECK_VALID_FILE(dictionary_file);

	string board_file = (fs::path(config_dir) / "board.yml").string();
	CHECK_VALID_FILE(board_file);

	CHECK_PARAM_EXISTS(parser, "req_num_frames");
	int required_number_of_frames = parser.get<int>("req_num_frames");
	CHECK_VALID_NUM_FRAMES(required_number_of_frames);

	bool preprocess = parser.has("preprocess");

	resolve_intrinsics(intrinsics_file, configuration);
	resolve_out_dir(out_dir, configuration);

	configuration.set_board_file(board_file);
	configuration.set_dictionary_file(dictionary_file);
	configuration.set_preprocessing(preprocess);
	
	configuration.set_in_depth_images_dir(depth_images_dir);
	configuration.set_in_rgb_images_dir(rgb_images_dir);
	configuration.set_marker_size(marker_size);
	configuration.set_voxel_size(voxel_size);
	configuration.set_required_number_of_frames(required_number_of_frames);

	return true;
}
