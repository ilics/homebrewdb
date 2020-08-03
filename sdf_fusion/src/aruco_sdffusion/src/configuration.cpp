//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <filesystem>
#include "configuration.hpp"

using namespace std;
namespace fs = std::filesystem;

string Configuration::get_in_depth_images_dir() const
{
	return in_depth_images_dir;
}

string Configuration::get_in_rgb_images_dir() const
{
	return in_rgb_images_dir;
}

float Configuration::get_marker_size() const
{
	return marker_size;
}

void Configuration::set_marker_size(float p_marker_size)
{
	marker_size = p_marker_size;
}

string Configuration::get_out_depth_images_dir() const
{
	return out_depth_images_dir;
}

string Configuration::get_out_rgb_images_dir() const
{
	return out_rgb_images_dir;
}

void Configuration::set_out_rgb_images_dir(const string& p_out_rgb_images_dir)
{
	out_rgb_images_dir = p_out_rgb_images_dir;
}

void Configuration::set_out_depth_images_dir(const string& p_out_depth_images_dir)
{
	out_depth_images_dir = p_out_depth_images_dir;
}

void Configuration::set_in_depth_images_dir(const string& p_in_depth_images_dir)
{
	in_depth_images_dir = p_in_depth_images_dir;
}

void Configuration::set_in_rgb_images_dir(const string& p_in_rgb_images_dir)
{
	in_rgb_images_dir = p_in_rgb_images_dir;
}

void Configuration::set_board_file(const string& p_board_file)
{
	board_file = p_board_file;
}

void Configuration::set_dictionary_file(const string& p_dictionary_file)
{
	dictionary_file = p_dictionary_file;
}


std::string Configuration::get_board_file() const
{
	return board_file;
}

std::string Configuration::get_dictionary_file() const
{
	return dictionary_file;
}


void Configuration::set_intrinsics(const Eigen::Matrix3f& p_intrinsics)
{
	intrinsics = p_intrinsics;
}

Eigen::Matrix3f Configuration::get_intrinsics() const
{
	return intrinsics;
}

string Configuration::get_out_dir() const
{
	return out_dir;
}

void Configuration::set_out_dir(const string& p_out_dir)
{
	out_dir = p_out_dir;
}

void Configuration::set_voxel_size(float p_voxel_size)
{
	voxel_size = p_voxel_size;
}

float Configuration::get_voxel_size() const
{
	return voxel_size;
}

int Configuration::get_required_number_of_frames() const
{
	return required_number_of_frames;
}

void Configuration::set_required_number_of_frames(int p_required_number_of_frames)
{
	required_number_of_frames = p_required_number_of_frames;
}

string Configuration::get_poses_file() const
{
	return (fs::path(out_dir) / "camera_poses.txt").string();
}

string Configuration::get_model_file() const
{
	return (fs::path(out_dir) / "model.ply").string();
}

bool Configuration::do_preprocessing() const
{
	return preprocessing;
}

void Configuration::set_preprocessing(bool p_preprocessing)
{
	preprocessing = p_preprocessing;
}


