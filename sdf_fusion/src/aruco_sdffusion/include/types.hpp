//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef TYPES_HPP
#define TYPES_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/LU>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

template <typename T>
struct array_deleter {
    void operator() (T const* p) {
        _mm_free((void*)p);
    }
};


template <typename T> void init( std::shared_ptr<T> data, T const val, int size );
template <> void init<float>( std::shared_ptr<float> data, float val, int size );
template <> void init<Eigen::Vector3f>( std::shared_ptr<Eigen::Vector3f> data, Eigen::Vector3f val, int size );

template <typename T>
struct voxel_cube {
    int size_x, size_y, size_z;
    float voxel_size;
    T default_value;
    //std::vector<T> data;
	std::shared_ptr<T> data;
    //std::valarray<T> data;

    voxel_cube(){}

    voxel_cube(int sx, int sy, int sz, float voxel_s, T const default_v) :
        size_x(sx), size_y(sy), size_z(sz), voxel_size(voxel_s), default_value(default_v) {
        //data.resize(sx * sy * sz, default_v);
		data = std::shared_ptr<T>( (T*) _mm_malloc( sx*sy*sz*sizeof(T), 16 ), array_deleter<T>() );
        if ( data == NULL ) std::cerr << "MEMORY ALLOCATION FAILED" << std::endl;
        init(data,default_v,sx*sy*sz);
    }

    //voxel_cube(voxel_cube<T> const& other) :
    //    size_x(other.size_x), size_y(other.size_y), size_z(other.size_z),
    //    voxel_size(other.voxel_size), default_value(other.default_value),data(other.data) {}

	inline T& at(int x, int y, int z) {return data.get()[x + size_x * y + size_x * size_y * z];}
    inline T at(int x, int y, int z) const {return data.get()[x + size_x * y + size_x * size_y * z];}

    inline T& at(int idx) {return data.get()[idx];}
    inline T at(int idx) const {return data.get()[idx];}

    operator std::string() const {
        std::stringstream out;
        for (T t : data) {
            out << t << " ";
        }
        return out.str();
    }

};

template <typename T>
voxel_cube<T> operator+( const voxel_cube<T>& left, const voxel_cube<T>& right ) {
    voxel_cube<T> result = left;
    for (int z = 0; z < result.size_z; ++z)
        for (int y = 0; y < result.size_y; ++y)
            for (int x = 0; x < result.size_x; ++x)
                result.at(x,y,z) += right.at(x,y,z);
    return result;
}

template <typename T>
void operator+= ( voxel_cube<T>& left, const voxel_cube<T>& right ) {
    for (int z = 0; z < left.size_z; ++z)
        for (int y = 0; y < left.size_y; ++y)
            for (int x = 0; x < left.size_x; ++x)
                left.at(x,y,z) += right.at(x,y,z);
}

template <typename T>
voxel_cube<T> operator-( const voxel_cube<T>& left, const voxel_cube<T>& right ) {
    voxel_cube<T> result = left;
    for (int z = 0; z < result.size_z; ++z)
        for (int y = 0; y < result.size_y; ++y)
            for (int x = 0; x < result.size_x; ++x)
                result.at(x,y,z) -= right.at(x,y,z);
    return result;
}

template <typename T>
voxel_cube<T> operator*( const voxel_cube<T>& left, const voxel_cube<T>& right ) {
    voxel_cube<T> result = left;
    for (int z = 0; z < result.size_z; ++z)
        for (int y = 0; y < result.size_y; ++y)
            for (int x = 0; x < result.size_x; ++x)
                result.at(x,y,z) *= right.at(x,y,z);
    return result;
}

template <typename T>
voxel_cube<T> operator/ ( voxel_cube<T>& grid, const voxel_cube<T>& divisor ) {
    voxel_cube<T> result = grid;
    for (int z = 0; z < result.size_z; ++z)
        for (int y = 0; y < result.size_y; ++y)
            for (int x = 0; x < result.size_x; ++x)
                result.at(x,y,z) /= divisor.at(x,y,z);
    return result;
}

template <typename T>
void operator/= ( voxel_cube<T>& grid, const T divisor ) {
    for (int z = 0; z < grid.size_z; ++z)
        for (int y = 0; y < grid.size_y; ++y)
            for (int x = 0; x < grid.size_x; ++x)
                grid.at(x,y,z) /= divisor;
}

template <typename T>
struct color_cube {

    voxel_cube<T> red, green, blue;

    color_cube() {}

    color_cube(voxel_cube<T> const& r,voxel_cube<T> const& g, voxel_cube<T>const& b)
        : red(r), green(g), blue(b) {}

    color_cube(color_cube const& o) : red(o.red), green(o.green), blue(o.blue) {}

    color_cube(int size_x, int size_y, int size_z, float voxel_size) :
        red  ( voxel_cube<T>(size_x, size_y, size_z, voxel_size, 0) ),
        green( voxel_cube<T>(size_x, size_y, size_z, voxel_size, 0) ),
        blue ( voxel_cube<T>(size_x, size_y, size_z, voxel_size, 0) ) {}
};

template <typename T>
color_cube<T> operator+( const color_cube<T>& left, const color_cube<T>& right ) {
    color_cube<T> result = left;
    result.red   += right.red;
    result.green += right.green;
    result.blue  += right.blue;
    return result;
}

template <typename T>
color_cube<T> operator-( const color_cube<T>& left, const color_cube<T>& right ) {
    color_cube<T> result = left;
    result.red   -= right.red;
    result.green -= right.green;
    result.blue  -= right.blue;
    return result;
}

template <typename T>
color_cube<T> operator*( const color_cube<T>& colors, const voxel_cube<T>& weights ) {
    color_cube<T> result = colors;
    result.red   *= weights;
    result.green *= weights;
    result.blue  *= weights;
    return result;
}

template <typename T>
void operator/= ( color_cube<T>& grid, const T divisor ) {
    grid.red   /= divisor;
    grid.green /= divisor;
    grid.blue  /= divisor;
}

struct sdf { // RGB SDF

    int size_x, size_y, size_z;
    float voxel_size;
    voxel_cube<float> distance_field, weights;
    color_cube<float> color_field;

    sdf() {}

    sdf(int x, int y, int z, float vox_size, float default_w=0.0f, float default_v=-1.0f) :
        size_x(x), size_y(y), size_z(z), voxel_size(vox_size),
        distance_field(voxel_cube<float>(x, y, z, vox_size, default_v)),
        weights(voxel_cube<float>(x, y, z, vox_size, default_w)),
        color_field(color_cube<float>(x,y,z,vox_size))
       { }

    sdf(voxel_cube<float> const& df, color_cube<float> const& cf)
        : size_x(df.size_x), size_y(df.size_y),size_z(df.size_z), voxel_size(df.voxel_size),
        distance_field(df), color_field(cf){}

    sdf(sdf const& o) : size_x(o.size_x), size_y(o.size_y), size_z(o.size_z), voxel_size(o.voxel_size),
    distance_field(o.distance_field), weights(o.weights), color_field(o.color_field){}

    float& weight(int x, int y, int z) {return weights.at(x,y,z);}

    float weight(int x, int y, int z) const {return weights.at(x,y,z);}

};

struct plain_sdf {
    int size_x, size_y, size_z;
    float voxel_size;
    voxel_cube<float> distance_field, weights;

	plain_sdf() {}

    plain_sdf(int x, int y, int z, float vox_size, float default_w, float default_v) :
        size_x(x), size_y(y), size_z(z), voxel_size(vox_size),
        distance_field(voxel_cube<float>(x, y, z, vox_size, default_v)),
        weights(voxel_cube<float>(x, y, z, vox_size, default_w))
       { }

    plain_sdf(plain_sdf const& o) : size_x(o.size_x), size_y(o.size_y), size_z(o.size_z), voxel_size(o.voxel_size),
    distance_field(o.distance_field), weights(o.weights) {}
};

#endif // TYPES_HPP
