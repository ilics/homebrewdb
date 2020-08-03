// Copyright (c) 2014, Tobias Holl
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//   1.   Redistributions of source code must retain the above copyright notice,
//        this list of conditions and the following disclaimer.
//   2.   Redistributions in binary form must reproduce the above copyright notice,
//        this list of conditions and the following disclaimer in the documentation
//        and/or other materials provided with the distribution.
//   3.   Neither the name of the copyright holder nor the names of its contributors
//        may be used to endorse or promote products derived from this software
//        without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

// modification: defines and mesh() code moved to separate file to avoid multiple inclusion
// saving PLY files, both in ASCII and binary formats, added to this file


#ifndef MARCHING_CUBES_HPP
#define MARCHING_CUBES_HPP

#include <memory>
#include <boost/progress.hpp>

#include "types.hpp"

namespace marching_cubes {

/**
 * @brief Vertex struct to manage handling points with normals
 */
struct vertex {
    Eigen::Vector3f point;
    Eigen::Vector3f normal;

    //int reference = -1; // If this vertex should be replaced by another one, store its ID here.
	int reference;

    // Color
    unsigned char r;
    unsigned char g;
    unsigned char b;

    // Location in the voxel cube
    int x;
    int y;
    int z;

	vertex(Eigen::Vector3f p, Eigen::Vector3f n, unsigned char rv, unsigned char gv, unsigned char bv, int xv, int yv, int zv) : point(p), normal(n), reference(-1), r(rv), g(gv), b(bv), x(xv), y(yv), z(zv) {}
    vertex(vertex const& o) : point(o.point), normal(o.normal), reference(o.reference), r(o.r), g(o.g), b(o.b), x(o.x), y(o.y), z(o.z) {}
    vertex(int ref) : reference(ref) {}

    // Equality and inequality operators for searching
    bool operator==(vertex other) {
        return ((reference == other.reference && reference != -1) ||
                (point == other.point && normal == other.normal));
    }
    bool operator!=(vertex other) {
        return ((reference != other.reference) ||
                (point != other.point) ||
                (normal != other.normal));
    }
};

/**
 * @brief Holds a triangle
 */
struct triangle {
    std::vector<vertex> points;

    triangle() {}
    triangle(std::vector<vertex> const& p) : points(p) {}
    triangle(triangle const& o) : points(o.points) {}
};

Eigen::Vector3f interpolate(Eigen::Vector3f p1, Eigen::Vector3f p2, float valp1, float valp2);
Eigen::Vector3f norm_at(sdf &s, int x, int y, int z);
std::vector<triangle> getCubeCase(sdf &s, int x, int y, int z);

} // namespace reconstruct::marching_cubes

/**
 * @brief mesh Mesh and scale a sdf object
 * @param s Finished SDF data
 */
std::vector<marching_cubes::triangle> mesh(sdf &s);
std::vector<marching_cubes::triangle> mesh(plain_sdf &s);
std::vector<marching_cubes::triangle> mesh_valid_only(sdf &s);

std::vector<marching_cubes::triangle> mesh_world(sdf &s, Eigen::Vector3f lower_left);



void save_mesh_ply(std::vector<marching_cubes::triangle> &tris, std::string const& filename, bool allow_duplicates=false );
void save_mesh_binary_ply( std::vector<marching_cubes::triangle> tris, std::string const& filename, bool allow_duplicates=false );
void save_mesh_binary_ply(std::vector<marching_cubes::triangle> tris, std::vector<marching_cubes::vertex> points, int element_vertex_count, std::string const& filename);

void save_point_cloud_ply( std::vector<Eigen::Vector3f> pts, std::vector<Eigen::Vector3f> rgbs, std::string const& filename );
void save_point_cloud_binary_ply( std::vector<Eigen::Vector3f> pts, std::vector<Eigen::Vector3f> rgbs, std::string const& filename );

void postprocess_triangles(std::vector<marching_cubes::triangle> &tris, bool allow_duplicates, std::vector<marching_cubes::vertex> &points, Eigen::Vector3f &point_coord_mean, int &element_vertex_count);


#endif // MARCHING_CUBES_HPP
