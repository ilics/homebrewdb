//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include <configuration.hpp>
#include <configuration_parser.hpp>
#include <iostream>
#include "sdf_fusion.hpp"

#define _USE_MATH_DEFINES

using namespace cv;
using namespace std;

void log_configuration(const Configuration &config)
{
	std::cout
		<< "Board file       = " << config.get_board_file() << std::endl
		<< "Dictionary file  = " << config.get_dictionary_file() << std::endl
		<< "In rgb images dir  = " << config.get_in_rgb_images_dir() << std::endl
		<< "In depth images dir  = " << config.get_in_depth_images_dir() << std::endl
		<< "Out rgb images dir  = " << config.get_out_rgb_images_dir() << std::endl
		<< "Out depth images dir  = " << config.get_out_depth_images_dir() << std::endl
		<< "Out dir  = " << config.get_out_dir() << std::endl
		<< "Marker size  = " << config.get_marker_size() << std::endl
	    << "Do preprocessing  = " << config.do_preprocessing() << std::endl
		<< "Intrinsics  = " << config.get_intrinsics() << std::endl;
	
}

int main(int argc, char** argv)
{
	Configuration configuration;

	bool configuration_resolved;
	try
	{
		configuration_resolved = resolve_configuration(argc, argv, configuration);
		
	} catch (const std::exception& e)
	{
		cerr << "Error parsing configuration: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	if (!configuration_resolved)
	{
		return EXIT_FAILURE;
	}

	log_configuration(configuration);
	
	if (run_sdf_fusion(configuration))
	{
		return EXIT_SUCCESS;
	}

	return EXIT_FAILURE;
}
