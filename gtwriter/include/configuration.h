//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_CONFIGURATION_H
#define GT_WRITER_CONFIGURATION_H

#include <Eigen/Core>

class Configuration {

public:

    const Eigen::Matrix3f& get_intrinsics() const
	{
		return intrinsics;
	}

	float get_focal_length_scale() const
	{
		return focal_length_scale;
	}

	void set_intrinsics(const Eigen::Matrix3f &p_intrinsics)
	{
		intrinsics = p_intrinsics;
	}

	void set_focal_length_scale(float p_focal_length_scale)
	{
		focal_length_scale = p_focal_length_scale;
	}

	void set_reference_models_dir(const std::string &p_reference_models_dir)
    {
		reference_models_dir = p_reference_models_dir;
    }

	std::string get_reference_models_dir() const
    {
		return reference_models_dir;
    }

	const std::vector<std::string>& get_scenes_dirs() const
    {
		return scenes_dirs;
    }

	void set_scenes_dirs(const std::vector<std::string> &p_scenes_dirs)
    {
		scenes_dirs = p_scenes_dirs;
    }

	bool get_copy_images() const
    {
		return copy_images;
    }

	void set_copy_images(bool p_copy_images)
    {
		copy_images = p_copy_images;
    }

	void set_out_dir(const std::string &p_out_dir)
    {
		out_dir = p_out_dir;
    }

	std::string get_out_dir() const
    {
		return out_dir;
    }

	int get_image_width() const
    {
		return image_width;
    }

	int get_image_height() const
    {
		return image_height;
    }
	
	void set_image_width(int p_width)
    {
		image_width = p_width;
    }

	void set_image_height(int p_height)
    {
		image_height = p_height;
    }

private:	
	Eigen::Matrix3f intrinsics;
	std::string reference_models_dir;
	std::vector<std::string> scenes_dirs;

	int image_width;
	int image_height;

	float focal_length_scale;
	std::string out_dir;
	bool copy_images;
};

#endif