//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_JOINT_SCENE
#define GT_WRITER_JOINT_SCENE

#include "scene.h"
#include <iostream>
#include "frame_model.h"

using namespace std;

class JointScene
{
public:

	JointScene(string p_common_name, vector<Scene> &p_scenes) : common_name(p_common_name), scenes(p_scenes)
	{}

	vector<vector<FrameModel>> to_frame_models()
	{
		vector<vector<FrameModel>> joint_frame_models;

		float max_deg = 0.0;
		float min_deg = 100000.0;
		
		for (auto &scene : scenes) {
			vector<vector<FrameModel>> scene_frame_models = scene.convert_to_frame_models();

			FrameModel &fm = scene_frame_models[0][0];

			if (fm.max_deg > max_deg)
			{
				max_deg = fm.max_deg;
			}
			if (fm.min_deg < min_deg)
			{
				min_deg = fm.min_deg;
			}

			joint_frame_models.insert(end(joint_frame_models), begin(scene_frame_models), end(scene_frame_models));
		}

		FrameModel &fm = joint_frame_models[0][0];
		fm.max_deg = max_deg;
		fm.min_deg = min_deg;
		return joint_frame_models;
	}

	vector<Scene> scenes;
	string common_name;
};

#endif