//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include "sdf_fusion.hpp"
#include <filesystem>
#include "input_preprocessor.hpp"
#include "reconstructor_3d.hpp"

using namespace std;
using namespace Eigen;

bool run_sdf_fusion(const Configuration &configuration)
{
	vector<float> volume(6, 0.f);
	const float marker_size = configuration.get_marker_size();

	volume[0] = -5.f * marker_size; volume[1] = 5.f * marker_size;
	volume[2] = -5.f * marker_size; volume[3] = 5.f * marker_size;
	volume[4] = -0.3f; volume[5] = 0.0f;

	if (configuration.do_preprocessing())
	{
		const bool preprocessed_successfully = preprocess_input(configuration, volume);

		if (!preprocessed_successfully)
		{
			cerr << "Input preprocessing failed, exiting" << endl;
			return false;
		}
	}

	return run_reconstruction(configuration, volume);
}


