//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <fstream>
#include <Eigen/Core>


template<typename T>
inline Eigen::Matrix<T, 3, 3> read_intrinsics_from_file(const std::string &in_file)
{
	std::vector<T> intrinsics_vector = read_matrix_one_line<T>(in_file);
	return matrix_from_vector_rowwise<T, 3, 3>(intrinsics_vector);
}

template<typename T>
inline Eigen::Matrix<T, 2, 1> project_point(const Eigen::Matrix<T, 3, 1> &point, const Eigen::Matrix<T, 3, 3> &intrinsics)
{
	Eigen::Matrix<T, 3, 1> projected = point / point(2);
	projected = intrinsics * projected;
	return { projected(0), projected(1) };
}


template<typename T, int R, int C>
inline Eigen::Matrix<T, R, C> matrix_from_vector_rowwise(std::vector<T> v)
{
	if (v.size() != R * C)
	{
		std::cerr << "Invalid input vector of size " << v.size() << " for " << R << "x" << C << " matrix" << std::endl;
		throw std::invalid_argument("Invalid vector size");
	}

	return Eigen::Map<Eigen::Matrix<T, R, C, Eigen::RowMajor>>(&v[0]);
}


template<typename T>
inline std::vector<T> read_matrix_one_line(const std::string &in_file)
{
	std::string line;
	std::vector<T> elements;

	std::ifstream f(in_file);

	while (std::getline(f, line))
	{
		stringstream ss(line);
		T current_value;

		while (ss >> current_value)
		{
			elements.push_back(current_value);
		}

	}

	return elements;
}

#endif
