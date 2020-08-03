//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef MATHTOOLS_H
#define MATHTOOLS_H


#define _USE_MATH_DEFINES
#include <math.h>

#define SQR(a) ((a)*(a))

namespace tr
{
	inline float fovToFocalLength(const float fov, const float length)
	{
		return (length / (2.0 * std::tan(fov / 2.0)));
	}

	inline float degrees_to_radians(const float deg)
	{
		return deg * M_PI / 180.0f;
	}
};

#endif