//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_SEQUENCE_COMPOSER
#define GT_WRITER_SEQUENCE_COMPOSER

#include "frame.h"
#include <iostream>
#include "configuration.h"

bool compose_joined_sequence(const Configuration &configuration, std::vector<Frame> &joined_sequences);


#endif