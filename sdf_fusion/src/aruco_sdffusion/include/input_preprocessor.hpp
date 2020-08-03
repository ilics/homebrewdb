//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef INPUT_PREPROCESSOR_HPP
#define INPUT_PREPROCESSOR_HPP

#include <iostream>
#include "configuration.hpp"

bool preprocess_input(const Configuration &configuration, const std::vector<float> &volume);

#endif
