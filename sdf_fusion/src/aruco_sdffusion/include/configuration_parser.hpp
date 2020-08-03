//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef CONFIGURATION_PARSER_HPP
#define CONFIGURATION_PARSER_HPP

#include "configuration.hpp"
#include <iostream>

bool resolve_configuration(int argc, char** argv, Configuration &configuration);

#endif