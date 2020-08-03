//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "types.hpp"

template <typename T> void init( std::shared_ptr<T> data, T const val, int size ) {
    for (int i = 0; i < size; ++i)
        data.get()[i] = val;
}

template <> void init<float>( std::shared_ptr<float> data, float val, int size ) {
    __m128* dst = (__m128*)( (void*)data.get() );
    for (int i = 0; i < (int)std::floor(size/4); ++i)
        dst[i] = _mm_set_ps1( val );
    for (int i = int( std::floor(size/4) )*4; i < size; ++i)
        data.get()[i] = val;
}

template <> void init<Eigen::Vector3f>( std::shared_ptr<Eigen::Vector3f> data, Eigen::Vector3f val, int size ) {
    for (int i = 0; i < size; ++i)
        data.get()[i] = val;
}