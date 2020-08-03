//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef REF_CONFIGURATION_PARSER
#define REF_CONFIGURATION_PARSER
#include "configuration.h"


bool parse_configuration(const std::string &config_file, Configuration &configuration);

#endif