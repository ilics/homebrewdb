//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_FRAME
#define GT_WRITER_FRAME

#include "frame_model.h"
#include <iostream>
class Frame
{	
public:
	int frame_id;
	std::vector<FrameModel> frame_models;
	std::string scene_dir;

};


#endif