//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef EDGE_ALIGNMENT_UTIL_H
#define EDGE_ALIGNMENT_UTIL_H

#include <opencv/cv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <iterator>
#include <Eigen/Core>
#include <Eigen/Dense>

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> operator-(const Eigen::Matrix<T, Eigen::Dynamic, 1> &vec, T s)
{
  return vec - s * Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Ones(vec.rows(), vec.cols());
}

template<typename T>
T get_median(const Eigen::Matrix<T, Eigen::Dynamic, 1> &v) {
  Eigen::Matrix<T, Eigen::Dynamic, 1> copy = v;
  const int len = copy.derived().size();

  if (len % 2 == 0) {
    std::nth_element(copy.data(), copy.data() + len / 2 - 1, copy.data() + len);
    T n1 = copy(len / 2 - 1);
    std::nth_element(copy.data(), copy.data() + len / 2, copy.data() + len);
    T n2 = copy(len / 2);
    return (n1 + n2) / static_cast<T>(2);

  } else {
    std::nth_element( copy.data(), copy.data() + len / 2, copy.data() + len);
    return copy(len / 2 );
  }
}


template<typename T>
inline Eigen::Matrix<T, 2, 1> project_point(const Eigen::Matrix<T, 3, 1> &point, const Eigen::Matrix<T, 3, 3> &intrinsics) {
  Eigen::Matrix<T, 3, 1> projected = point / point(2);
  projected = intrinsics * projected;
  return {projected(0), projected(1)};
}


template<typename T>
inline Eigen::Transform<T, 3, Eigen::Isometry> to_isometry(const Eigen::Matrix<T, 4, 4> &pose_matrix) {
  Eigen::Transform<T, 3, Eigen::Isometry> pose;
  pose.setIdentity();

  pose.linear() = pose_matrix.block(0, 0, 3, 3);
  pose.translation() = pose_matrix.block(0, 3, 3, 1);

  return pose;
}

inline cv::Mat read_image(const std::string &in_file) {
  cv::Mat image_rgb = cv::imread(in_file.c_str(), cv::IMREAD_COLOR);
  cv::Mat image;
  if (!image_rgb.data)                              // Check for invalid input
  {
    std::cout << "Could not open or find the image" << std::endl;

  }
  return image_rgb;
}

inline Eigen::Matrix4d create_transformation(const Eigen::Matrix3d &rotation_matrix, const Eigen::Vector3d &translation_vector) {
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

  transformation.block<3, 3>(0, 0) = rotation_matrix;
  transformation.block<3, 1>(0, 3) = translation_vector;
  return transformation;
}

inline Eigen::Vector2f get_edge_normal(const Eigen::Vector4f &line_endpoints) {
  float v_x = line_endpoints[0] - line_endpoints[2];
  float v_y = line_endpoints[1] - line_endpoints[3];

  float mag = sqrt(v_x * v_x + v_y * v_y);
  float nv_x = v_x / mag;
  float nv_y = v_y / mag;

  // 90 degree rotation
  float temp = nv_x;
  nv_x = -nv_y;
  nv_y = temp;

  return {nv_x, nv_y};
}


#endif //EDGE_ALIGNMENT_UTIL_H