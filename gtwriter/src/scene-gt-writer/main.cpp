//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################


#include <Eigen/Core>
#include "util.h"
#include <nlohmann/json.hpp>
#include "io_utils.hpp"
#include <filesystem>
#include "sequence_composer.h"
#include "output_writer.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

using namespace std;
using namespace Eigen;


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

bool parse_scenes_dirs(const string &scenes_dirs_string, vector<string> &scenes_dirs)
{
	stringstream scenes_ss(scenes_dirs_string);	
	try
	{
		while (scenes_ss.good())
		{
			string scene_dir;
			getline(scenes_ss, scene_dir, ',');
			CHECK_VALID_DIR(scene_dir)
			scenes_dirs.push_back(scene_dir);
		}
	}
	catch (const std::exception&)
	{
		cerr << "Invalid scenes dirs: " << scenes_dirs_string << endl;
		return false;
	}

	if (scenes_dirs.empty())
	{
		cerr << "No scene directories provided" << endl;
		return false;
	}

	return true;
	
}

bool get_image_size(const Configuration &configuration, int &width, int &height)
{
	const vector<string>& scene_dirs = configuration.get_scenes_dirs();
	
	string first_dir = scene_dirs[0];
	fs::path rgb_dir = (fs::path(first_dir) / "images" / "rgb");

	//pick the first file
	string filename = get_image_filename_for_idx(0);
	string first_file = (rgb_dir / filename).string();
	CHECK_VALID_FILE(first_file)

	cv::Mat first_image = cv::imread(first_file);
	
	height = first_image.rows;
	width = first_image.cols;

	return height != 0 && width != 0;
}

bool parse_configuration(const cv::CommandLineParser &parser, Configuration &configuration)
{
	CHECK_PARAM_EXISTS(parser, "intrinsics_file")
	string intrinsics_file = parser.get<string>("intrinsics_file");
	CHECK_VALID_FILE(intrinsics_file)
	Matrix3f intrinsics = read_intrinsics_from_file<float>(intrinsics_file);

	CHECK_PARAM_EXISTS(parser, "reference_models_dir")
	string reference_models_dir = parser.get<string>("reference_models_dir");
	CHECK_VALID_DIR(reference_models_dir)

	CHECK_PARAM_EXISTS(parser, "out_dir")
	string out_dir = parser.get<string>("out_dir");

	CHECK_PARAM_EXISTS(parser, "scenes_dirs")
	string scenes_dirs_string = parser.get<string>("scenes_dirs");

	vector<string> scenes_dirs;
	if (!parse_scenes_dirs(scenes_dirs_string, scenes_dirs)) return false;
	
	configuration.set_scenes_dirs(scenes_dirs);

	bool copy_images = parser.has("copy_images");

	int image_width, image_height;

	if (!get_image_size(configuration, image_width, image_height)) return false;

	configuration.set_intrinsics(intrinsics);
	configuration.set_reference_models_dir(reference_models_dir);
	configuration.set_out_dir(out_dir);
	configuration.set_focal_length_scale(0.5f);
	configuration.set_copy_images(copy_images);
	configuration.set_scenes_dirs(scenes_dirs);
	configuration.set_image_height(image_height);
	configuration.set_image_width(image_width);
	
	return true;
}

int main(int argc, char* argv[])
{
	const cv::String arg_keys =
		"{help h |      | help message   }"
		"{intrinsics_file       |     | intrinsics file         }"
		"{reference_models_dir           |     | directory with reference models         }"
		"{scenes_dirs           |     | comma separated directories to be joined         }"
		"{out_dir           |     | output directory         }"
		"{copy_images           |     | should copy and reindex images         }";

	cv::CommandLineParser parser(argc, argv, arg_keys);

	if (parser.has("help")) {
		parser.printMessage();
		return EXIT_SUCCESS;
	}

	Configuration configuration;
	if (!parse_configuration(parser, configuration)) return EXIT_FAILURE;

	vector<Frame> joined_sequences;
	if (!compose_joined_sequence(configuration, joined_sequences)) return EXIT_FAILURE;

	write_output(configuration, joined_sequences);
	
	return 0;
}
