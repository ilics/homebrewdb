//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_OUTPUT_WRITER
#define GT_WRITER_OUTPUT_WRITER
#include <iostream>
#include "frame.h"
#include "configuration.h"

void write_output(const Configuration &configuration, const std::vector<Frame> &joined_sequences);

#endif
