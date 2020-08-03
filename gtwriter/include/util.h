//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_UTIL_H
#define GT_WRITER_UTIL_H

#include <iostream>
#include <string>
#include <fstream>
#include "stdlib.h"
#include <iomanip>
#include <iterator>
#include "Eigen/Core"


inline std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
	str.erase(0, str.find_first_not_of(chars));
	return str;
}

inline std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
	str.erase(str.find_last_not_of(chars) + 1);
	return str;
}

inline std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
	return ltrim(rtrim(str, chars), chars);
}

inline std::string get_image_filename_for_idx(size_t idx)
{
	std::stringstream filename_ss;
	filename_ss << std::setfill('0') << std::setw(6) << idx << ".png";
	std::string filename = filename_ss.str();
	return filename;
}


#endif