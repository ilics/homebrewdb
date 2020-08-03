//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <iostream>
#include <types.hpp>
#include <Eigen/Core>

class Configuration
{
  public:
	  void set_out_dir(const std::string &p_out_dir);
	  std::string get_out_dir() const;

	  void set_in_depth_images_dir(const std::string &p_in_depth_images_dir);
	  std::string get_in_depth_images_dir() const;

	  void set_in_rgb_images_dir(const std::string &p_in_rgb_images_dir);
	  std::string get_in_rgb_images_dir() const;

	  void set_out_depth_images_dir(const std::string &p_out_depth_images_dir);
	  std::string get_out_depth_images_dir() const;

	  void set_out_rgb_images_dir(const std::string &p_rgb_images_dir);
	  std::string get_out_rgb_images_dir() const;

	  void set_board_file(const std::string &p_board_file);
	  std::string get_board_file() const;

	  void set_dictionary_file(const std::string &p_dictionary_file);
	  std::string get_dictionary_file() const;

	  void set_intrinsics(const Eigen::Matrix3f &p_intrinsics);
	  Eigen::Matrix3f get_intrinsics() const;

	  void set_marker_size(float p_marker_size);
	  float get_marker_size() const;

	  void set_voxel_size(float voxel_size);
	  float get_voxel_size() const;

	  void set_required_number_of_frames(int p_required_number_of_frames);
	  int get_required_number_of_frames() const;

	  std::string get_poses_file() const;
	  std::string get_model_file() const;

	  bool do_preprocessing() const;
	  void set_preprocessing(bool p_preprocessing);

  private:
	  std::string dictionary_file;
	  std::string board_file;
	  std::string out_dir;

	  std::string in_rgb_images_dir;
	  std::string in_depth_images_dir;

	  std::string out_rgb_images_dir;
	  std::string out_depth_images_dir;

	  float marker_size;
	  float voxel_size;

	  Eigen::Matrix3f intrinsics;

	  int required_number_of_frames;
	  bool preprocessing;
};

#endif