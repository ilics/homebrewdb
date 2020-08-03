//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_MODEL_H
#define GT_WRITER_MODEL_H
#include <iostream>
#include <Eigen/Core>

class Model {

public:

	Model(const std::string p_model_file, const Eigen::Matrix4f &p_canonical_pose, int model_id) : model_file(p_model_file), canonical_pose(p_canonical_pose), model_id(model_id)
	{}

	std::string model_file;
	Eigen::Matrix4f canonical_pose;
	int model_id;	
};

#endif
