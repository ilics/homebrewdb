//######################################################################
//#   PPFshapematchinghalcon Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef SHAPE_MATCHING_CONFIGURATION_HPP
#define SHAPE_MATCHING_CONFIGURATION_HPP
#include <iostream>

class Configuration
{

public:
	
	void set_reference_models_dir(const std::string &p_reference_models_dir)
	{
		reference_models_dir = p_reference_models_dir;
	}

	void set_scene_dir(const std::string &p_scene_dir)
	{
		scene_dir = p_scene_dir;
	}

	void set_model_ids(const std::vector<int> &p_model_ids)
	{
		model_ids = p_model_ids;
	}

	void set_relative_sampling_distance(double p_relative_sampling_distance)
	{
		relative_sampling_distance = p_relative_sampling_distance;
	}

	void set_keypoint_fraction(double p_keypoint_fraction)
	{
		keypoint_fraction = p_keypoint_fraction;
	}

	const std::string get_reference_models_dir() const
	{
		return reference_models_dir;
	}

	const std::string get_scene_dir() const
	{
		return scene_dir;
	}

	const std::vector<int> get_model_ids() const
	{
		return model_ids;
	}

	const double get_relative_sampling_distance() const
	{
		return relative_sampling_distance;
	}

	const double get_keypoint_fraction() const
	{
		return keypoint_fraction;
	}


private:
	std::string reference_models_dir;
	std::string scene_dir;
	std::vector<int> model_ids;

	double relative_sampling_distance;
	double keypoint_fraction;

};


#endif