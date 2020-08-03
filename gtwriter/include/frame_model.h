//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_FRAME_MODEL_H
#define GT_WRITER_FRAME_MODEL_H
#include <Eigen/Core>
#include <iostream>
#include "model.h"


class FrameModel
{

public:
	FrameModel(int p_model_id, const Eigen::Matrix4f &p_pose, const Eigen::Vector4i &p_bbox) : model_id(p_model_id), pose(p_pose), bbox(p_bbox){}

	int model_id;
	Eigen::Matrix4f pose;
	Eigen::Vector4i bbox;
};

#endif
