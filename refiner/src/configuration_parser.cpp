//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "configuration_parser.h"
#include <filesystem>
#include <nlohmann/json.hpp>
#include <fstream>
#include "io_utils.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace std;


#define CHECK_PARAM_EXISTS(config_json, param_name) \
               if (config_json.find(param_name) == config_json.end()) {\
                  cerr << "Missing parameter: " << param_name << endl;\
                  return false;\
               }

#define CHECK_VALID_DIR(dir) \
      if (!fs::is_directory(dir)) {\
        cerr << "Invalid directory: " << dir << endl;\
        return false;\
      }


bool parse_configuration(const std::string& config_file, Configuration& configuration)
{
	if (!fs::exists(config_file))
	{
		cerr << "Config file " << config_file << " does not exist!" << endl;
		return false;
	}

	std::ifstream is(config_file);
	json config_json;

	is >> config_json;

	CHECK_PARAM_EXISTS(config_json, "intrinsics_file")
	auto intrinsics_file = config_json["intrinsics_file"].get<string>();

	CHECK_PARAM_EXISTS(config_json, "model_padding_pixels")
	auto model_padding_pixels = config_json["model_padding_pixels"].get<int>();

	CHECK_PARAM_EXISTS(config_json, "edge_search_span")
	auto edge_search_span = config_json["edge_search_span"].get<int>();

	CHECK_PARAM_EXISTS(config_json, "point_sampling_step")
	auto point_sampling_step = config_json["point_sampling_step"].get<int>();

	CHECK_PARAM_EXISTS(config_json, "max_iterations")
	auto max_iterations = config_json["max_iterations"].get<int>();

	CHECK_PARAM_EXISTS(config_json, "reference_models_dir")
	auto reference_models_dir = config_json["reference_models_dir"].get<string>();
	CHECK_VALID_DIR(reference_models_dir)

	CHECK_PARAM_EXISTS(config_json, "occlusion_threshold")
	auto occlusion_threshold = config_json["occlusion_threshold"].get<float>();

	CHECK_PARAM_EXISTS(config_json, "object_visibility_threshold")
	auto object_visibility_threshold = config_json["object_visibility_threshold"].get<float>();

	configuration.set_intrinsics(read_intrinsics_from_file<float>(intrinsics_file));
	configuration.set_model_padding_pixels(model_padding_pixels);
	configuration.set_edge_search_span(edge_search_span);
	configuration.set_point_sampling_step(point_sampling_step);
	configuration.set_max_interations(max_iterations);
	configuration.set_model_reference_dir(reference_models_dir);
	configuration.set_object_visibility_thredhold(object_visibility_threshold);
	configuration.set_occlusion_threshold(occlusion_threshold);

	if (!config_json["z_near"].is_null())
	{
		configuration.set_z_near(config_json["z_near"].get<float>());
	}

	if (!config_json["z_far"].is_null())
	{
		configuration.set_z_far(config_json["z_far"].get<float>());
	}

	return true;
}
