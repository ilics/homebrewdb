//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iostream>
#include <opencv/cv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

class Configuration {
public:


  Eigen::Matrix3f get_intrinsics() const {
    return intrinsics;
  }

  int get_model_padding_pixels() const {
    return model_padding_pixels;
  }

  int get_edge_search_span() const {
    return edge_search_span;
  }


  int get_max_iterations() const {
    return max_iterations;
  }

  int get_point_sampling_step() const {
    return point_sampling_step;
  }


  float get_z_far() const {
    return z_far;
  }

  float get_z_near() const {
    return z_near;
  }


  void set_intrinsics(const Eigen::Matrix3f &p_intrinsics) {
    intrinsics = p_intrinsics;
  }


  std::string get_reference_models_dir() const {
    return reference_models_dir;
  }

  float get_occlusion_threshold() const {
	return occlusion_threshold;
  }

  float get_object_visibility_threshold() const {
	return object_visibility_threshold;
  }

  void set_model_padding_pixels(int padding_pixels) {
    model_padding_pixels = padding_pixels;
  }

  void set_edge_search_span(int search_span) {
    edge_search_span = search_span;
  }


  void set_max_interations(int iterations) {
    max_iterations = iterations;
  }

  void set_point_sampling_step(int step) {
    point_sampling_step = step;
  }

  void set_z_near(float p_z_near) {
    z_near = p_z_near;
  }

  void set_z_far(float p_z_far) {
    z_far = p_z_far;
  }

  void set_model_reference_dir(const std::string &p_reference_models_dir) {
    reference_models_dir = p_reference_models_dir;
  }

  void set_occlusion_threshold(float p_occlusion_threshold)
  {
	occlusion_threshold = p_occlusion_threshold;
  }

  void set_object_visibility_thredhold(float p_object_visibility_thredhold)
  {
	object_visibility_threshold = p_object_visibility_thredhold;
  }


private:
  std::string reference_models_dir;
  Eigen::Matrix3f intrinsics;
  int model_padding_pixels;
  int edge_search_span;
  int point_sampling_step;
  int max_iterations;

// how many meters difference in depth means occlision
  float occlusion_threshold;
// fraction of object pixels 
  float object_visibility_threshold;

  float z_near;
  float z_far;
};

#endif
