//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#pragma once

#define SQR(a) ((a)*(a))

namespace camera
{
	inline float fovToFocalLength(const float fov, const float length)
	{
		return (length / (2.0 * std::tan(fov / 2.0)));
	}
};