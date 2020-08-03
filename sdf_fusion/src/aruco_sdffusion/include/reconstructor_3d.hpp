//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef RECONSTRUCTOR3D_HPP
#define RECONSTRUCTOR3D_HPP
#include <iostream>
#include "configuration.hpp"

bool run_reconstruction(const Configuration &configuration, const std::vector<float> &volume);

#endif