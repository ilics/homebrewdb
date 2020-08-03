//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################\

#include "marching_cubes.hpp"
#include <fstream>

namespace marching_cubes {

/**
 * Marching cubes. Implementation adapted from code by
 * Cory Bloyd (in the public domain), available at
 * http://paulbourke.net/geometry/polygonise/marchingsource.cpp
 */

// Data for marching cubes algorithm

int edge_table[256] = {
    0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

int tri_table[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  1,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  8,  3,  9,  8,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  3,  1,  2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 9,  2, 10,  0,  2,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 2,  8,  3,  2, 10,  8, 10,  9,  8, -1, -1, -1, -1, -1, -1, -1},
    { 3, 11,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0, 11,  2,  8, 11,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  9,  0,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1, 11,  2,  1,  9, 11,  9,  8, 11, -1, -1, -1, -1, -1, -1, -1},
    { 3, 10,  1, 11, 10,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0, 10,  1,  0,  8, 10,  8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    { 3,  9,  0,  3, 11,  9, 11, 10,  9, -1, -1, -1, -1, -1, -1, -1},
    { 9,  8, 10, 10,  8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  7,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  3,  0,  7,  3,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  1,  9,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  1,  9,  4,  7,  1,  7,  3,  1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 10,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 3,  4,  7,  3,  0,  4,  1,  2, 10, -1, -1, -1, -1, -1, -1, -1},
    { 9,  2, 10,  9,  0,  2,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1},
    { 2, 10,  9,  2,  9,  7,  2,  7,  3,  7,  9,  4, -1, -1, -1, -1},
    { 8,  4,  7,  3, 11,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11,  4,  7, 11,  2,  4,  2,  0,  4, -1, -1, -1, -1, -1, -1, -1},
    { 9,  0,  1,  8,  4,  7,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1},
    { 4,  7, 11,  9,  4, 11,  9, 11,  2,  9,  2,  1, -1, -1, -1, -1},
    { 3, 10,  1,  3, 11, 10,  7,  8,  4, -1, -1, -1, -1, -1, -1, -1},
    { 1, 11, 10,  1,  4, 11,  1,  0,  4,  7, 11,  4, -1, -1, -1, -1},
    { 4,  7,  8,  9,  0, 11,  9, 11, 10, 11,  0,  3, -1, -1, -1, -1},
    { 4,  7, 11,  4, 11,  9,  9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    { 9,  5,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 9,  5,  4,  0,  8,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  5,  4,  1,  5,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 8,  5,  4,  8,  3,  5,  3,  1,  5, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 10,  9,  5,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 3,  0,  8,  1,  2, 10,  4,  9,  5, -1, -1, -1, -1, -1, -1, -1},
    { 5,  2, 10,  5,  4,  2,  4,  0,  2, -1, -1, -1, -1, -1, -1, -1},
    { 2, 10,  5,  3,  2,  5,  3,  5,  4,  3,  4,  8, -1, -1, -1, -1},
    { 9,  5,  4,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0, 11,  2,  0,  8, 11,  4,  9,  5, -1, -1, -1, -1, -1, -1, -1},
    { 0,  5,  4,  0,  1,  5,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1},
    { 2,  1,  5,  2,  5,  8,  2,  8, 11,  4,  8,  5, -1, -1, -1, -1},
    {10,  3, 11, 10,  1,  3,  9,  5,  4, -1, -1, -1, -1, -1, -1, -1},
    { 4,  9,  5,  0,  8,  1,  8, 10,  1,  8, 11, 10, -1, -1, -1, -1},
    { 5,  4,  0,  5,  0, 11,  5, 11, 10, 11,  0,  3, -1, -1, -1, -1},
    { 5,  4,  8,  5,  8, 10, 10,  8, 11, -1, -1, -1, -1, -1, -1, -1},
    { 9,  7,  8,  5,  7,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 9,  3,  0,  9,  5,  3,  5,  7,  3, -1, -1, -1, -1, -1, -1, -1},
    { 0,  7,  8,  0,  1,  7,  1,  5,  7, -1, -1, -1, -1, -1, -1, -1},
    { 1,  5,  3,  3,  5,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 9,  7,  8,  9,  5,  7, 10,  1,  2, -1, -1, -1, -1, -1, -1, -1},
    {10,  1,  2,  9,  5,  0,  5,  3,  0,  5,  7,  3, -1, -1, -1, -1},
    { 8,  0,  2,  8,  2,  5,  8,  5,  7, 10,  5,  2, -1, -1, -1, -1},
    { 2, 10,  5,  2,  5,  3,  3,  5,  7, -1, -1, -1, -1, -1, -1, -1},
    { 7,  9,  5,  7,  8,  9,  3, 11,  2, -1, -1, -1, -1, -1, -1, -1},
    { 9,  5,  7,  9,  7,  2,  9,  2,  0,  2,  7, 11, -1, -1, -1, -1},
    { 2,  3, 11,  0,  1,  8,  1,  7,  8,  1,  5,  7, -1, -1, -1, -1},
    {11,  2,  1, 11,  1,  7,  7,  1,  5, -1, -1, -1, -1, -1, -1, -1},
    { 9,  5,  8,  8,  5,  7, 10,  1,  3, 10,  3, 11, -1, -1, -1, -1},
    { 5,  7,  0,  5,  0,  9,  7, 11,  0,  1,  0, 10, 11, 10,  0, -1},
    {11, 10,  0, 11,  0,  3, 10,  5,  0,  8,  0,  7,  5,  7,  0, -1},
    {11, 10,  5,  7, 11,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  3,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 9,  0,  1,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  8,  3,  1,  9,  8,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1},
    { 1,  6,  5,  2,  6,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  6,  5,  1,  2,  6,  3,  0,  8, -1, -1, -1, -1, -1, -1, -1},
    { 9,  6,  5,  9,  0,  6,  0,  2,  6, -1, -1, -1, -1, -1, -1, -1},
    { 5,  9,  8,  5,  8,  2,  5,  2,  6,  3,  2,  8, -1, -1, -1, -1},
    { 2,  3, 11, 10,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11,  0,  8, 11,  2,  0, 10,  6,  5, -1, -1, -1, -1, -1, -1, -1},
    { 0,  1,  9,  2,  3, 11,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1},
    { 5, 10,  6,  1,  9,  2,  9, 11,  2,  9,  8, 11, -1, -1, -1, -1},
    { 6,  3, 11,  6,  5,  3,  5,  1,  3, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8, 11,  0, 11,  5,  0,  5,  1,  5, 11,  6, -1, -1, -1, -1},
    { 3, 11,  6,  0,  3,  6,  0,  6,  5,  0,  5,  9, -1, -1, -1, -1},
    { 6,  5,  9,  6,  9, 11, 11,  9,  8, -1, -1, -1, -1, -1, -1, -1},
    { 5, 10,  6,  4,  7,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  3,  0,  4,  7,  3,  6,  5, 10, -1, -1, -1, -1, -1, -1, -1},
    { 1,  9,  0,  5, 10,  6,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1},
    {10,  6,  5,  1,  9,  7,  1,  7,  3,  7,  9,  4, -1, -1, -1, -1},
    { 6,  1,  2,  6,  5,  1,  4,  7,  8, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2,  5,  5,  2,  6,  3,  0,  4,  3,  4,  7, -1, -1, -1, -1},
    { 8,  4,  7,  9,  0,  5,  0,  6,  5,  0,  2,  6, -1, -1, -1, -1},
    { 7,  3,  9,  7,  9,  4,  3,  2,  9,  5,  9,  6,  2,  6,  9, -1},
    { 3, 11,  2,  7,  8,  4, 10,  6,  5, -1, -1, -1, -1, -1, -1, -1},
    { 5, 10,  6,  4,  7,  2,  4,  2,  0,  2,  7, 11, -1, -1, -1, -1},
    { 0,  1,  9,  4,  7,  8,  2,  3, 11,  5, 10,  6, -1, -1, -1, -1},
    { 9,  2,  1,  9, 11,  2,  9,  4, 11,  7, 11,  4,  5, 10,  6, -1},
    { 8,  4,  7,  3, 11,  5,  3,  5,  1,  5, 11,  6, -1, -1, -1, -1},
    { 5,  1, 11,  5, 11,  6,  1,  0, 11,  7, 11,  4,  0,  4, 11, -1},
    { 0,  5,  9,  0,  6,  5,  0,  3,  6, 11,  6,  3,  8,  4,  7, -1},
    { 6,  5,  9,  6,  9, 11,  4,  7,  9,  7, 11,  9, -1, -1, -1, -1},
    {10,  4,  9,  6,  4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4, 10,  6,  4,  9, 10,  0,  8,  3, -1, -1, -1, -1, -1, -1, -1},
    {10,  0,  1, 10,  6,  0,  6,  4,  0, -1, -1, -1, -1, -1, -1, -1},
    { 8,  3,  1,  8,  1,  6,  8,  6,  4,  6,  1, 10, -1, -1, -1, -1},
    { 1,  4,  9,  1,  2,  4,  2,  6,  4, -1, -1, -1, -1, -1, -1, -1},
    { 3,  0,  8,  1,  2,  9,  2,  4,  9,  2,  6,  4, -1, -1, -1, -1},
    { 0,  2,  4,  4,  2,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 8,  3,  2,  8,  2,  4,  4,  2,  6, -1, -1, -1, -1, -1, -1, -1},
    {10,  4,  9, 10,  6,  4, 11,  2,  3, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  2,  2,  8, 11,  4,  9, 10,  4, 10,  6, -1, -1, -1, -1},
    { 3, 11,  2,  0,  1,  6,  0,  6,  4,  6,  1, 10, -1, -1, -1, -1},
    { 6,  4,  1,  6,  1, 10,  4,  8,  1,  2,  1, 11,  8, 11,  1, -1},
    { 9,  6,  4,  9,  3,  6,  9,  1,  3, 11,  6,  3, -1, -1, -1, -1},
    { 8, 11,  1,  8,  1,  0, 11,  6,  1,  9,  1,  4,  6,  4,  1, -1},
    { 3, 11,  6,  3,  6,  0,  0,  6,  4, -1, -1, -1, -1, -1, -1, -1},
    { 6,  4,  8, 11,  6,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 7, 10,  6,  7,  8, 10,  8,  9, 10, -1, -1, -1, -1, -1, -1, -1},
    { 0,  7,  3,  0, 10,  7,  0,  9, 10,  6,  7, 10, -1, -1, -1, -1},
    {10,  6,  7,  1, 10,  7,  1,  7,  8,  1,  8,  0, -1, -1, -1, -1},
    {10,  6,  7, 10,  7,  1,  1,  7,  3, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2,  6,  1,  6,  8,  1,  8,  9,  8,  6,  7, -1, -1, -1, -1},
    { 2,  6,  9,  2,  9,  1,  6,  7,  9,  0,  9,  3,  7,  3,  9, -1},
    { 7,  8,  0,  7,  0,  6,  6,  0,  2, -1, -1, -1, -1, -1, -1, -1},
    { 7,  3,  2,  6,  7,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 2,  3, 11, 10,  6,  8, 10,  8,  9,  8,  6,  7, -1, -1, -1, -1},
    { 2,  0,  7,  2,  7, 11,  0,  9,  7,  6,  7, 10,  9, 10,  7, -1},
    { 1,  8,  0,  1,  7,  8,  1, 10,  7,  6,  7, 10,  2,  3, 11, -1},
    {11,  2,  1, 11,  1,  7, 10,  6,  1,  6,  7,  1, -1, -1, -1, -1},
    { 8,  9,  6,  8,  6,  7,  9,  1,  6, 11,  6,  3,  1,  3,  6, -1},
    { 0,  9,  1, 11,  6,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 7,  8,  0,  7,  0,  6,  3, 11,  0, 11,  6,  0, -1, -1, -1, -1},
    { 7, 11,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 7,  6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 3,  0,  8, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  1,  9, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 8,  1,  9,  8,  3,  1, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1},
    {10,  1,  2,  6, 11,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 10,  3,  0,  8,  6, 11,  7, -1, -1, -1, -1, -1, -1, -1},
    { 2,  9,  0,  2, 10,  9,  6, 11,  7, -1, -1, -1, -1, -1, -1, -1},
    { 6, 11,  7,  2, 10,  3, 10,  8,  3, 10,  9,  8, -1, -1, -1, -1},
    { 7,  2,  3,  6,  2,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 7,  0,  8,  7,  6,  0,  6,  2,  0, -1, -1, -1, -1, -1, -1, -1},
    { 2,  7,  6,  2,  3,  7,  0,  1,  9, -1, -1, -1, -1, -1, -1, -1},
    { 1,  6,  2,  1,  8,  6,  1,  9,  8,  8,  7,  6, -1, -1, -1, -1},
    {10,  7,  6, 10,  1,  7,  1,  3,  7, -1, -1, -1, -1, -1, -1, -1},
    {10,  7,  6,  1,  7, 10,  1,  8,  7,  1,  0,  8, -1, -1, -1, -1},
    { 0,  3,  7,  0,  7, 10,  0, 10,  9,  6, 10,  7, -1, -1, -1, -1},
    { 7,  6, 10,  7, 10,  8,  8, 10,  9, -1, -1, -1, -1, -1, -1, -1},
    { 6,  8,  4, 11,  8,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 3,  6, 11,  3,  0,  6,  0,  4,  6, -1, -1, -1, -1, -1, -1, -1},
    { 8,  6, 11,  8,  4,  6,  9,  0,  1, -1, -1, -1, -1, -1, -1, -1},
    { 9,  4,  6,  9,  6,  3,  9,  3,  1, 11,  3,  6, -1, -1, -1, -1},
    { 6,  8,  4,  6, 11,  8,  2, 10,  1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 10,  3,  0, 11,  0,  6, 11,  0,  4,  6, -1, -1, -1, -1},
    { 4, 11,  8,  4,  6, 11,  0,  2,  9,  2, 10,  9, -1, -1, -1, -1},
    {10,  9,  3, 10,  3,  2,  9,  4,  3, 11,  3,  6,  4,  6,  3, -1},
    { 8,  2,  3,  8,  4,  2,  4,  6,  2, -1, -1, -1, -1, -1, -1, -1},
    { 0,  4,  2,  4,  6,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  9,  0,  2,  3,  4,  2,  4,  6,  4,  3,  8, -1, -1, -1, -1},
    { 1,  9,  4,  1,  4,  2,  2,  4,  6, -1, -1, -1, -1, -1, -1, -1},
    { 8,  1,  3,  8,  6,  1,  8,  4,  6,  6, 10,  1, -1, -1, -1, -1},
    {10,  1,  0, 10,  0,  6,  6,  0,  4, -1, -1, -1, -1, -1, -1, -1},
    { 4,  6,  3,  4,  3,  8,  6, 10,  3,  0,  3,  9, 10,  9,  3, -1},
    {10,  9,  4,  6, 10,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  9,  5,  7,  6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  3,  4,  9,  5, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1},
    { 5,  0,  1,  5,  4,  0,  7,  6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11,  7,  6,  8,  3,  4,  3,  5,  4,  3,  1,  5, -1, -1, -1, -1},
    { 9,  5,  4, 10,  1,  2,  7,  6, 11, -1, -1, -1, -1, -1, -1, -1},
    { 6, 11,  7,  1,  2, 10,  0,  8,  3,  4,  9,  5, -1, -1, -1, -1},
    { 7,  6, 11,  5,  4, 10,  4,  2, 10,  4,  0,  2, -1, -1, -1, -1},
    { 3,  4,  8,  3,  5,  4,  3,  2,  5, 10,  5,  2, 11,  7,  6, -1},
    { 7,  2,  3,  7,  6,  2,  5,  4,  9, -1, -1, -1, -1, -1, -1, -1},
    { 9,  5,  4,  0,  8,  6,  0,  6,  2,  6,  8,  7, -1, -1, -1, -1},
    { 3,  6,  2,  3,  7,  6,  1,  5,  0,  5,  4,  0, -1, -1, -1, -1},
    { 6,  2,  8,  6,  8,  7,  2,  1,  8,  4,  8,  5,  1,  5,  8, -1},
    { 9,  5,  4, 10,  1,  6,  1,  7,  6,  1,  3,  7, -1, -1, -1, -1},
    { 1,  6, 10,  1,  7,  6,  1,  0,  7,  8,  7,  0,  9,  5,  4, -1},
    { 4,  0, 10,  4, 10,  5,  0,  3, 10,  6, 10,  7,  3,  7, 10, -1},
    { 7,  6, 10,  7, 10,  8,  5,  4, 10,  4,  8, 10, -1, -1, -1, -1},
    { 6,  9,  5,  6, 11,  9, 11,  8,  9, -1, -1, -1, -1, -1, -1, -1},
    { 3,  6, 11,  0,  6,  3,  0,  5,  6,  0,  9,  5, -1, -1, -1, -1},
    { 0, 11,  8,  0,  5, 11,  0,  1,  5,  5,  6, 11, -1, -1, -1, -1},
    { 6, 11,  3,  6,  3,  5,  5,  3,  1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 10,  9,  5, 11,  9, 11,  8, 11,  5,  6, -1, -1, -1, -1},
    { 0, 11,  3,  0,  6, 11,  0,  9,  6,  5,  6,  9,  1,  2, 10, -1},
    {11,  8,  5, 11,  5,  6,  8,  0,  5, 10,  5,  2,  0,  2,  5, -1},
    { 6, 11,  3,  6,  3,  5,  2, 10,  3, 10,  5,  3, -1, -1, -1, -1},
    { 5,  8,  9,  5,  2,  8,  5,  6,  2,  3,  8,  2, -1, -1, -1, -1},
    { 9,  5,  6,  9,  6,  0,  0,  6,  2, -1, -1, -1, -1, -1, -1, -1},
    { 1,  5,  8,  1,  8,  0,  5,  6,  8,  3,  8,  2,  6,  2,  8, -1},
    { 1,  5,  6,  2,  1,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  3,  6,  1,  6, 10,  3,  8,  6,  5,  6,  9,  8,  9,  6, -1},
    {10,  1,  0, 10,  0,  6,  9,  5,  0,  5,  6,  0, -1, -1, -1, -1},
    { 0,  3,  8,  5,  6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10,  5,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11,  5, 10,  7,  5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11,  5, 10, 11,  7,  5,  8,  3,  0, -1, -1, -1, -1, -1, -1, -1},
    { 5, 11,  7,  5, 10, 11,  1,  9,  0, -1, -1, -1, -1, -1, -1, -1},
    {10,  7,  5, 10, 11,  7,  9,  8,  1,  8,  3,  1, -1, -1, -1, -1},
    {11,  1,  2, 11,  7,  1,  7,  5,  1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  3,  1,  2,  7,  1,  7,  5,  7,  2, 11, -1, -1, -1, -1},
    { 9,  7,  5,  9,  2,  7,  9,  0,  2,  2, 11,  7, -1, -1, -1, -1},
    { 7,  5,  2,  7,  2, 11,  5,  9,  2,  3,  2,  8,  9,  8,  2, -1},
    { 2,  5, 10,  2,  3,  5,  3,  7,  5, -1, -1, -1, -1, -1, -1, -1},
    { 8,  2,  0,  8,  5,  2,  8,  7,  5, 10,  2,  5, -1, -1, -1, -1},
    { 9,  0,  1,  5, 10,  3,  5,  3,  7,  3, 10,  2, -1, -1, -1, -1},
    { 9,  8,  2,  9,  2,  1,  8,  7,  2, 10,  2,  5,  7,  5,  2, -1},
    { 1,  3,  5,  3,  7,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  7,  0,  7,  1,  1,  7,  5, -1, -1, -1, -1, -1, -1, -1},
    { 9,  0,  3,  9,  3,  5,  5,  3,  7, -1, -1, -1, -1, -1, -1, -1},
    { 9,  8,  7,  5,  9,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 5,  8,  4,  5, 10,  8, 10, 11,  8, -1, -1, -1, -1, -1, -1, -1},
    { 5,  0,  4,  5, 11,  0,  5, 10, 11, 11,  3,  0, -1, -1, -1, -1},
    { 0,  1,  9,  8,  4, 10,  8, 10, 11, 10,  4,  5, -1, -1, -1, -1},
    {10, 11,  4, 10,  4,  5, 11,  3,  4,  9,  4,  1,  3,  1,  4, -1},
    { 2,  5,  1,  2,  8,  5,  2, 11,  8,  4,  5,  8, -1, -1, -1, -1},
    { 0,  4, 11,  0, 11,  3,  4,  5, 11,  2, 11,  1,  5,  1, 11, -1},
    { 0,  2,  5,  0,  5,  9,  2, 11,  5,  4,  5,  8, 11,  8,  5, -1},
    { 9,  4,  5,  2, 11,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 2,  5, 10,  3,  5,  2,  3,  4,  5,  3,  8,  4, -1, -1, -1, -1},
    { 5, 10,  2,  5,  2,  4,  4,  2,  0, -1, -1, -1, -1, -1, -1, -1},
    { 3, 10,  2,  3,  5, 10,  3,  8,  5,  4,  5,  8,  0,  1,  9, -1},
    { 5, 10,  2,  5,  2,  4,  1,  9,  2,  9,  4,  2, -1, -1, -1, -1},
    { 8,  4,  5,  8,  5,  3,  3,  5,  1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  4,  5,  1,  0,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 8,  4,  5,  8,  5,  3,  9,  0,  5,  0,  3,  5, -1, -1, -1, -1},
    { 9,  4,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4, 11,  7,  4,  9, 11,  9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    { 0,  8,  3,  4,  9,  7,  9, 11,  7,  9, 10, 11, -1, -1, -1, -1},
    { 1, 10, 11,  1, 11,  4,  1,  4,  0,  7,  4, 11, -1, -1, -1, -1},
    { 3,  1,  4,  3,  4,  8,  1, 10,  4,  7,  4, 11, 10, 11,  4, -1},
    { 4, 11,  7,  9, 11,  4,  9,  2, 11,  9,  1,  2, -1, -1, -1, -1},
    { 9,  7,  4,  9, 11,  7,  9,  1, 11,  2, 11,  1,  0,  8,  3, -1},
    {11,  7,  4, 11,  4,  2,  2,  4,  0, -1, -1, -1, -1, -1, -1, -1},
    {11,  7,  4, 11,  4,  2,  8,  3,  4,  3,  2,  4, -1, -1, -1, -1},
    { 2,  9, 10,  2,  7,  9,  2,  3,  7,  7,  4,  9, -1, -1, -1, -1},
    { 9, 10,  7,  9,  7,  4, 10,  2,  7,  8,  7,  0,  2,  0,  7, -1},
    { 3,  7, 10,  3, 10,  2,  7,  4, 10,  1, 10,  0,  4,  0, 10, -1},
    { 1, 10,  2,  8,  7,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  9,  1,  4,  1,  7,  7,  1,  3, -1, -1, -1, -1, -1, -1, -1},
    { 4,  9,  1,  4,  1,  7,  0,  8,  1,  8,  7,  1, -1, -1, -1, -1},
    { 4,  0,  3,  7,  4,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 4,  8,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 9, 10,  8, 10, 11,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 3,  0,  9,  3,  9, 11, 11,  9, 10, -1, -1, -1, -1, -1, -1, -1},
    { 0,  1, 10,  0, 10,  8,  8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    { 3,  1, 10, 11,  3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  2, 11,  1, 11,  9,  9, 11,  8, -1, -1, -1, -1, -1, -1, -1},
    { 3,  0,  9,  3,  9, 11,  1,  2,  9,  2, 11,  9, -1, -1, -1, -1},
    { 0,  2, 11,  8,  0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 3,  2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 2,  3,  8,  2,  8, 10, 10,  8,  9, -1, -1, -1, -1, -1, -1, -1},
    { 9, 10,  2,  0,  9,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 2,  3,  8,  2,  8, 10,  0,  1,  8,  1, 10,  8, -1, -1, -1, -1},
    { 1, 10,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 1,  3,  8,  9,  1,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  9,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    { 0,  3,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

Eigen::Vector3f interpolate(Eigen::Vector3f p1, Eigen::Vector3f p2, float valp1, float valp2) {
    if (std::abs(valp1) < 0.00001)
        return p1;
    if (std::abs(valp2) < 0.00001)
        return p2;
    if (std::abs(valp1 - valp2) < 0.00001)
        return p1;
    double mu = (- valp1) / (valp2 - valp1);
    return p1 + (p2 - p1) * mu;
}

Eigen::Vector3f norm_at(sdf &s, int x, int y, int z) {
    Eigen::Vector3f norm = 0.5f * Eigen::Vector3f(s.distance_field.at(x - 1, y, z) - s.distance_field.at(x + 1, y, z),
                                            s.distance_field.at(x, y - 1, z) - s.distance_field.at(x, y + 1, z),
                                            s.distance_field.at(x, y, z - 1) - s.distance_field.at(x, y, z + 1));
    norm.normalize();
    return norm * (-1);
}

Eigen::Vector3f norm_at(plain_sdf &s, int x, int y, int z) {
    Eigen::Vector3f norm = 0.5f * Eigen::Vector3f(s.distance_field.at(x - 1, y, z) - s.distance_field.at(x + 1, y, z),
                                            s.distance_field.at(x, y - 1, z) - s.distance_field.at(x, y + 1, z),
                                            s.distance_field.at(x, y, z - 1) - s.distance_field.at(x, y, z + 1));
    norm.normalize();
    return norm * (-1);
}

std::vector<triangle> getCubeCase(sdf &s, int x, int y, int z) {

    float v0 = s.distance_field.at(x, y, z);
    float v1 = s.distance_field.at(x + 1, y, z);
    float v2 = s.distance_field.at(x + 1, y + 1, z);
    float v3 = s.distance_field.at(x, y + 1, z);
    float v4 = s.distance_field.at(x, y, z + 1);
    float v5 = s.distance_field.at(x + 1, y, z + 1);
    float v6 = s.distance_field.at(x + 1, y + 1, z + 1);
    float v7 = s.distance_field.at(x, y + 1, z + 1);
    int index = 0;
    if (v0 > 0) index |= 1;
    if (v1 > 0) index |= 2;
    if (v2 > 0) index |= 4;
    if (v3 > 0) index |= 8;
    if (v4 > 0) index |= 16;
    if (v5 > 0) index |= 32;
    if (v6 > 0) index |= 64;
    if (v7 > 0) index |= 128;

    int edges = edge_table[index];
    int *trigs = tri_table[index];
    Eigen::Vector3f vertlist[12];
    Eigen::Vector3f normlist[12];


    /* Cube is entirely in/out of the surface */
    if (edges == 0)
        return std::vector<triangle>();

    /* Find the vertices where the surface intersects the cube */
    if (edges & 1)
        vertlist[0] = interpolate(Eigen::Vector3f(x, y, z),Eigen::Vector3f(x + 1, y, z),v0,v1),
        normlist[0] = interpolate(norm_at(s, x, y, z), norm_at(s, x + 1, y, z), v0, v1);
    if (edges & 2)
        vertlist[1] = interpolate(Eigen::Vector3f(x + 1, y, z),Eigen::Vector3f(x + 1, y + 1, z),v1,v2),
        normlist[1] = interpolate(norm_at(s, x + 1, y, z),norm_at(s, x + 1, y + 1, z),v1,v2);
    if (edges & 4)
        vertlist[2] = interpolate(Eigen::Vector3f(x + 1, y + 1, z),Eigen::Vector3f(x, y + 1, z),v2,v3),
        normlist[2] = interpolate(norm_at(s, x + 1, y + 1, z),norm_at(s, x, y + 1, z),v2,v3);
    if (edges & 8)
        vertlist[3] = interpolate(Eigen::Vector3f(x, y + 1, z),Eigen::Vector3f(x, y, z),v3,v0),
        normlist[3] = interpolate(norm_at(s, x, y + 1, z),norm_at(s, x, y, z),v3,v0);
    if (edges & 16)
        vertlist[4] = interpolate(Eigen::Vector3f(x, y, z + 1),Eigen::Vector3f(x + 1, y, z + 1),v4,v5),
        normlist[4] = interpolate(norm_at(s, x, y, z + 1),norm_at(s, x + 1, y, z + 1),v4,v5);
    if (edges & 32)
        vertlist[5] = interpolate(Eigen::Vector3f(x + 1, y, z + 1),Eigen::Vector3f(x + 1, y + 1, z + 1),v5,v6),
        normlist[5] = interpolate(norm_at(s, x + 1, y, z + 1),norm_at(s, x + 1, y + 1, z + 1),v5,v6);
    if (edges & 64)
        vertlist[6] = interpolate(Eigen::Vector3f(x + 1, y + 1, z + 1),Eigen::Vector3f(x, y + 1, z + 1),v6,v7),
        normlist[6] = interpolate(norm_at(s, x + 1, y + 1, z + 1),norm_at(s, x, y + 1, z + 1),v6,v7);
    if (edges & 128)
        vertlist[7] = interpolate(Eigen::Vector3f(x, y + 1, z + 1),Eigen::Vector3f(x, y, z + 1),v7,v4),
        normlist[7] = interpolate(norm_at(s, x, y + 1, z + 1),norm_at(s, x, y, z + 1),v7,v4);
    if (edges & 256)
        vertlist[8] = interpolate(Eigen::Vector3f(x, y, z),Eigen::Vector3f(x, y, z + 1),v0,v4),
        normlist[8] = interpolate(norm_at(s, x, y, z),norm_at(s, x, y, z + 1),v0,v4);
    if (edges & 512)
        vertlist[9] = interpolate(Eigen::Vector3f(x + 1, y, z),Eigen::Vector3f(x + 1, y, z + 1),v1,v5),
        normlist[9] = interpolate(norm_at(s, x + 1, y, z),norm_at(s, x + 1, y, z + 1),v1,v5);
    if (edges & 1024)
        vertlist[10] = interpolate(Eigen::Vector3f(x + 1, y + 1, z),Eigen::Vector3f(x + 1, y + 1, z + 1),v2,v6),
        normlist[10] = interpolate(norm_at(s, x + 1, y + 1, z),norm_at(s, x + 1, y + 1, z + 1),v2,v6);
    if (edges & 2048)
        vertlist[11] = interpolate(Eigen::Vector3f(x, y + 1, z),Eigen::Vector3f(x, y + 1, z + 1),v3,v7),
        normlist[11] = interpolate(norm_at(s, x, y + 1, z),norm_at(s, x, y + 1, z + 1),v3,v7);

    // Get colours
    int ri = (int) (s.color_field.red.at(x, y, z) * 255.0f);
    int gi = (int) (s.color_field.green.at(x, y, z) * 255.0f);
    int bi = (int) (s.color_field.blue.at(x, y, z) * 255.0f);
	//int ri = 100, gi = 100, bi = 100;
    // Limit to range from 0 to 255
    uchar r = (uchar) ((ri > 255) ? 255 : ((ri < 0) ? 0 : ri));
    uchar g = (uchar) ((gi > 255) ? 255 : ((gi < 0) ? 0 : gi));
    uchar b = (uchar) ((bi > 255) ? 255 : ((bi < 0) ? 0 : bi));

    // Create the triangles
    std::vector<triangle> triangles;
    for (int i = 0; trigs[i] != -1; i += 3) {
        std::vector<vertex> tri;
        tri.push_back(vertex(vertlist[trigs[i]], normlist[trigs[i]], r, g, b, x, y, z));
        tri.push_back(vertex(vertlist[trigs[i + 1]], normlist[trigs[i + 1]], r, g, b, x, y, z));
        tri.push_back(vertex(vertlist[trigs[i + 2]], normlist[trigs[i + 2]], r, g, b, x, y, z));
        triangles.emplace_back(tri);
    }
    return triangles;
}

std::vector<triangle> getCubeCase(plain_sdf &s, int x, int y, int z) {

    float v0 = s.distance_field.at(x, y, z);
    float v1 = s.distance_field.at(x + 1, y, z);
    float v2 = s.distance_field.at(x + 1, y + 1, z);
    float v3 = s.distance_field.at(x, y + 1, z);
    float v4 = s.distance_field.at(x, y, z + 1);
    float v5 = s.distance_field.at(x + 1, y, z + 1);
    float v6 = s.distance_field.at(x + 1, y + 1, z + 1);
    float v7 = s.distance_field.at(x, y + 1, z + 1);
    int index = 0;
    if (v0 > 0) index |= 1;
    if (v1 > 0) index |= 2;
    if (v2 > 0) index |= 4;
    if (v3 > 0) index |= 8;
    if (v4 > 0) index |= 16;
    if (v5 > 0) index |= 32;
    if (v6 > 0) index |= 64;
    if (v7 > 0) index |= 128;

    int edges = edge_table[index];
    int *trigs = tri_table[index];
    Eigen::Vector3f vertlist[12];
    Eigen::Vector3f normlist[12];


    /* Cube is entirely in/out of the surface */
    if (edges == 0)
        return std::vector<triangle>();

    /* Find the vertices where the surface intersects the cube */
    if (edges & 1)
        vertlist[0] = interpolate(Eigen::Vector3f(x, y, z),Eigen::Vector3f(x + 1, y, z),v0,v1),
        normlist[0] = interpolate(norm_at(s, x, y, z), norm_at(s, x + 1, y, z), v0, v1);
    if (edges & 2)
        vertlist[1] = interpolate(Eigen::Vector3f(x + 1, y, z),Eigen::Vector3f(x + 1, y + 1, z),v1,v2),
        normlist[1] = interpolate(norm_at(s, x + 1, y, z),norm_at(s, x + 1, y + 1, z),v1,v2);
    if (edges & 4)
        vertlist[2] = interpolate(Eigen::Vector3f(x + 1, y + 1, z),Eigen::Vector3f(x, y + 1, z),v2,v3),
        normlist[2] = interpolate(norm_at(s, x + 1, y + 1, z),norm_at(s, x, y + 1, z),v2,v3);
    if (edges & 8)
        vertlist[3] = interpolate(Eigen::Vector3f(x, y + 1, z),Eigen::Vector3f(x, y, z),v3,v0),
        normlist[3] = interpolate(norm_at(s, x, y + 1, z),norm_at(s, x, y, z),v3,v0);
    if (edges & 16)
        vertlist[4] = interpolate(Eigen::Vector3f(x, y, z + 1),Eigen::Vector3f(x + 1, y, z + 1),v4,v5),
        normlist[4] = interpolate(norm_at(s, x, y, z + 1),norm_at(s, x + 1, y, z + 1),v4,v5);
    if (edges & 32)
        vertlist[5] = interpolate(Eigen::Vector3f(x + 1, y, z + 1),Eigen::Vector3f(x + 1, y + 1, z + 1),v5,v6),
        normlist[5] = interpolate(norm_at(s, x + 1, y, z + 1),norm_at(s, x + 1, y + 1, z + 1),v5,v6);
    if (edges & 64)
        vertlist[6] = interpolate(Eigen::Vector3f(x + 1, y + 1, z + 1),Eigen::Vector3f(x, y + 1, z + 1),v6,v7),
        normlist[6] = interpolate(norm_at(s, x + 1, y + 1, z + 1),norm_at(s, x, y + 1, z + 1),v6,v7);
    if (edges & 128)
        vertlist[7] = interpolate(Eigen::Vector3f(x, y + 1, z + 1),Eigen::Vector3f(x, y, z + 1),v7,v4),
        normlist[7] = interpolate(norm_at(s, x, y + 1, z + 1),norm_at(s, x, y, z + 1),v7,v4);
    if (edges & 256)
        vertlist[8] = interpolate(Eigen::Vector3f(x, y, z),Eigen::Vector3f(x, y, z + 1),v0,v4),
        normlist[8] = interpolate(norm_at(s, x, y, z),norm_at(s, x, y, z + 1),v0,v4);
    if (edges & 512)
        vertlist[9] = interpolate(Eigen::Vector3f(x + 1, y, z),Eigen::Vector3f(x + 1, y, z + 1),v1,v5),
        normlist[9] = interpolate(norm_at(s, x + 1, y, z),norm_at(s, x + 1, y, z + 1),v1,v5);
    if (edges & 1024)
        vertlist[10] = interpolate(Eigen::Vector3f(x + 1, y + 1, z),Eigen::Vector3f(x + 1, y + 1, z + 1),v2,v6),
        normlist[10] = interpolate(norm_at(s, x + 1, y + 1, z),norm_at(s, x + 1, y + 1, z + 1),v2,v6);
    if (edges & 2048)
        vertlist[11] = interpolate(Eigen::Vector3f(x, y + 1, z),Eigen::Vector3f(x, y + 1, z + 1),v3,v7),
        normlist[11] = interpolate(norm_at(s, x, y + 1, z),norm_at(s, x, y + 1, z + 1),v3,v7);

    // Get colours
	int ri = 100, gi = 100, bi = 100;
    // Limit to range from 0 to 255
    uchar r = (uchar) ((ri > 255) ? 255 : ((ri < 0) ? 0 : ri));
    uchar g = (uchar) ((gi > 255) ? 255 : ((gi < 0) ? 0 : gi));
    uchar b = (uchar) ((bi > 255) ? 255 : ((bi < 0) ? 0 : bi));

    // Create the triangles
    std::vector<triangle> triangles;
    for (int i = 0; trigs[i] != -1; i += 3) {
        std::vector<vertex> tri;
        tri.push_back(vertex(vertlist[trigs[i]], normlist[trigs[i]], r, g, b, x, y, z));
        tri.push_back(vertex(vertlist[trigs[i + 1]], normlist[trigs[i + 1]], r, g, b, x, y, z));
        tri.push_back(vertex(vertlist[trigs[i + 2]], normlist[trigs[i + 2]], r, g, b, x, y, z));
        triangles.emplace_back(tri);
    }
    return triangles;
}

} // namespace reconstruct::marching_cubes

/**
 * @brief mesh Mesh and scale a sdf object
 * @param s Finished SDF data
 */
std::vector<marching_cubes::triangle> mesh(sdf &s) {
    // Create temporary storage for triangles
    std::vector<marching_cubes::triangle> triangles;
    // Mesh the voxel cubes
    for (int x = 1; x < s.size_x - 2; ++x) {
        for (int y = 1; y < s.size_y - 2; ++y) {
            for (int z = 1; z < s.size_z - 2; ++z) {
                std::vector<marching_cubes::triangle> res = marching_cubes::getCubeCase(s, x, y, z);
                triangles.insert(triangles.end(), res.begin(), res.end());
            }
        }
    }
    // Create output for transformed triangles
    std::vector<marching_cubes::triangle> tris_out;
    Eigen::Vector3f cube(s.size_x*s.voxel_size,s.size_y*s.voxel_size,s.size_z*s.voxel_size);
//	std::cout << s.voxel_size << ", " << s.size_x << " " << s.size_y << " " << s.size_z << std::endl;

//	float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::min();
//	float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::min();
//	float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::min();

    // Transform every triangle
    for (marching_cubes::triangle tri : triangles) {
        // Add triangle
        tris_out.emplace_back();
        // Transform triangle and add points to last element of tris_out
        for (marching_cubes::vertex &v : tri.points) {
            Eigen::Vector3f p = v.point * s.voxel_size - cube*0.5f;
//			Eigen::Vector3f p = v.point * s.voxel_size;
//			min_x = std::min(min_x,p(0)); max_x = std::max(max_x,p(0));
//			min_y = std::min(min_y,p(1)); max_y = std::max(max_y,p(1));
//			min_z = std::min(min_z,p(2)); max_z = std::max(max_z,p(2));
			//std::cout << p.transpose() << std::endl;
            tris_out.back().points.push_back(marching_cubes::vertex(p, v.normal, v.r, v.g, v.b, v.x, v.y, v.z));
        }
    }
//	std::cout << min_x << " " << max_x << std::endl;
//	std::cout << min_y << " " << max_y << std::endl;
//	std::cout << min_z << " " << max_z << std::endl;
    // Return
    return tris_out;
}

std::vector<marching_cubes::triangle> mesh_valid_only(sdf &s) {
	//Eigen::Array3f zero_array = Eigen::Array3f::Zero();
	//Eigen::Array3f one_array = Eigen::Array3f::Ones();

    // Create temporary storage for triangles
    std::vector<marching_cubes::triangle> triangles;
    // Mesh the voxel cubes
    for (int x = 1; x < s.size_x - 2; ++x) {
        for (int y = 1; y < s.size_y - 2; ++y) {
            for (int z = 1; z < s.size_z - 2; ++z) {
				//Eigen::Vector3f grad = 0.5f * Eigen::Vector3f( s.distance_field.at(x+1,y,z) - s.distance_field.at(x-1,y,z),
				//											   s.distance_field.at(x,y+1,z) - s.distance_field.at(x,y-1,z),
				//											   s.distance_field.at(x,y,z+1) - s.distance_field.at(x,y,z-1) );

				bool flag = true;
				for (int zi = -1; zi <= 1; ++zi)
					for (int yi = -1; yi <= 1; ++yi)
						for (int xi = -1; xi <= 1; ++xi)
							if ( !(s.weights.at(x+xi,y+yi,z+zi) > 0.f) )
								flag = false;

				//if ( (grad.cwiseAbs().array() > zero_array).all() && (grad.cwiseAbs().array() < one_array).all() && s.weights.at(x,y,z) > 0.f ) {
				//if ( (grad.cwiseAbs().array() < one_array).all() && s.weights.at(x,y,z) > 0.f ) {
				//if ( (grad.cwiseAbs().array() > zero_array).all() && (grad.cwiseAbs().array() < one_array).all() ) {
				//if ( (grad.cwiseAbs().array() < one_array).all() ) {
				//if ( s.weights.at(x,y,z) > 0.f ) {
				if ( flag ) {
					std::vector<marching_cubes::triangle> res = marching_cubes::getCubeCase(s, x, y, z);
					triangles.insert(triangles.end(), res.begin(), res.end());
				}
            }
        }
    }
    // Create output for transformed triangles
    std::vector<marching_cubes::triangle> tris_out;
    Eigen::Vector3f cube(0.5f * s.size_x*s.voxel_size,0.5f * s.size_y*s.voxel_size,s.size_z*s.voxel_size);

    // Transform every triangle
    for (marching_cubes::triangle tri : triangles) {
        // Add triangle
        tris_out.emplace_back();
        // Transform triangle and add points to last element of tris_out
        for (marching_cubes::vertex &v : tri.points) {
			Eigen::Vector3f p = v.point * s.voxel_size - cube;
//			Eigen::Vector3f p = v.point * s.voxel_size;
            tris_out.back().points.push_back(marching_cubes::vertex(p, v.normal, v.r, v.g, v.b, v.x, v.y, v.z));
        }
    }

	// Return
    return tris_out;
}

std::vector<marching_cubes::triangle> mesh(plain_sdf &s) {
    // Create temporary storage for triangles
    std::vector<marching_cubes::triangle> triangles;
    // Mesh the voxel cubes
    for (int x = 1; x < s.size_x - 2; ++x) {
        for (int y = 1; y < s.size_y - 2; ++y) {
            for (int z = 1; z < s.size_z - 2; ++z) {
                std::vector<marching_cubes::triangle> res = marching_cubes::getCubeCase(s, x, y, z);
                triangles.insert(triangles.end(), res.begin(), res.end());
            }
        }
    }
    // Create output for transformed triangles
    std::vector<marching_cubes::triangle> tris_out;
    Eigen::Vector3f cube(s.size_x*s.voxel_size,s.size_y*s.voxel_size,s.size_z*s.voxel_size);

    // Transform every triangle
    for (marching_cubes::triangle tri : triangles) {
        // Add triangle
        tris_out.emplace_back();
        // Transform triangle and add points to last element of tris_out
        for (marching_cubes::vertex &v : tri.points) {
            Eigen::Vector3f p = v.point * s.voxel_size - cube*0.5f;
            tris_out.back().points.push_back(marching_cubes::vertex(p, v.normal, v.r, v.g, v.b, v.x, v.y, v.z));
        }
    }
    return tris_out;
}

std::vector<marching_cubes::triangle> mesh_world(sdf &s, Eigen::Vector3f lower_left) {
	// Create temporary storage for triangles
    std::vector<marching_cubes::triangle> triangles;
    // Mesh the voxel cubes
    for (int x = 1; x < s.size_x - 2; ++x) {
        for (int y = 1; y < s.size_y - 2; ++y) {
            for (int z = 1; z < s.size_z - 2; ++z) {
                std::vector<marching_cubes::triangle> res = marching_cubes::getCubeCase(s, x, y, z);
                triangles.insert(triangles.end(), res.begin(), res.end());
            }
        }
    }
    // Create output for transformed triangles
    std::vector<marching_cubes::triangle> tris_out;
    Eigen::Vector3f cube(s.size_x*s.voxel_size,s.size_y*s.voxel_size,s.size_z*s.voxel_size);

    // Transform every triangle
    for (marching_cubes::triangle &tri : triangles) {
        // Add triangle
        tris_out.emplace_back();
        // Transform triangle and add points to last element of tris_out
        for (marching_cubes::vertex &v : tri.points) {
            Eigen::Vector3f p = v.point * s.voxel_size + lower_left;
            tris_out.back().points.push_back(marching_cubes::vertex(p, v.normal, v.r, v.g, v.b, v.x, v.y, v.z));
        }
    }

    // Return
    return tris_out;
}

void save_mesh_ply( std::vector<marching_cubes::triangle> &tris, std::string const& filename, bool allow_duplicates ) {
    // Open output file
    std::ofstream file(filename);
    if(!file.is_open()) {
        std::cerr << "File " << filename << " can not be opened." << std::endl;
        throw std::logic_error("Could not open output file.");
    }

    std::vector<marching_cubes::vertex> points;
    int count = 0;
    if (!allow_duplicates) {
        std::cout << "Cleaning output (" << tris.size() * 3 << " vertices)" << std::endl;
        boost::progress_display progress(tris.size());
        // Get points without duplicates
        for (unsigned int i = 0; i < tris.size(); ++i) {
            std::vector<marching_cubes::vertex>& pts = tris.at(i).points;
            for (unsigned int j = 0; j < pts.size(); ++j) {
                marching_cubes::vertex& p = pts.at(j);
                auto where = std::find(points.begin(), points.end(), p); // Where this point may be found.
                if (where == points.end()) {
                    p.reference = count++;
                    points.push_back(p);
                } else {
                    // Mark as a duplicate in the triangle.
                    // Vector iterator is a random access iterator, so this is legal.
                    // Will not compile otherwise (std::distance would work, but its
                    // complexity increases on other containers...)
                    p.reference = where - points.begin();
                }
            }
            ++progress;
        }
    } else {
        for (unsigned int i = 0; i < tris.size(); ++i) {
            std::vector<marching_cubes::vertex>& pts = tris.at(i).points;
            for (unsigned int j = 0; j < pts.size(); ++j) {
                points.push_back(pts.at(j));
                pts.at(j).reference = count++;
            }
        }
    }

    std::cout << "Writing output to " << filename << "..." << std::endl;

    // Write header
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << count << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property float nx" << std::endl;
    file << "property float ny" << std::endl;
    file << "property float nz" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "element face " << tris.size() << std::endl;
    file << "property list uchar int vertex_indices" << std::endl;
    file << "end_header" << std::endl;

    // Write points (removing duplicates)
    for (auto pt_it = points.begin(); pt_it != points.end(); ++pt_it) {
        file << pt_it->point(0) << " " << pt_it->point(1) << " " << pt_it->point(2) << " ";
        file << pt_it->normal(0) << " " << pt_it->normal(1) << " " << pt_it->normal(2) << " ";
        file << (int) pt_it->r << " " << (int) pt_it->g << " " << (int) pt_it->b << std::endl;
    }

    // Write triangles
    for (auto tri_it = tris.begin(); tri_it != tris.end(); ++tri_it) {
        file << "3 " << tri_it->points.at(0).reference << " "
                     << tri_it->points.at(1).reference << " "
                     << tri_it->points.at(2).reference << std::endl;
    }

    file.close();
    std::cout << "Mesh successfully outputted." << std::endl;
}

void postprocess_triangles(std::vector<marching_cubes::triangle> &tris, bool allow_duplicates, std::vector<marching_cubes::vertex> &points, Eigen::Vector3f &point_coord_mean, int &element_vertex_count)
{
	element_vertex_count = 0;
	if (!allow_duplicates) {
		std::cout << "Cleaning output (" << tris.size() * 3 << " vertices)" << std::endl;
		boost::progress_display progress(tris.size());
		// Get points without duplicates
		for (unsigned int i = 0; i < tris.size(); ++i) {
			std::vector<marching_cubes::vertex>& pts = tris.at(i).points;
			for (unsigned int j = 0; j < pts.size(); ++j) {
				marching_cubes::vertex& p = pts.at(j);
				auto where = std::find(points.begin(), points.end(), p); // Where this point may be found.
				if (where == points.end()) {
					p.reference = element_vertex_count++;
					points.push_back(p);
				}
				else {
					// Mark as a duplicate in the triangle.
					// Vector iterator is a random access iterator, so this is legal.
					// Will not compile otherwise (std::distance would work, but its
					// complexity increases on other containers...)
					p.reference = where - points.begin();
				}
			}
			++progress;
		}
	}
	else {
		for (unsigned int i = 0; i < tris.size(); ++i) {
			std::vector<marching_cubes::vertex>& pts = tris.at(i).points;
			for (unsigned int j = 0; j < pts.size(); ++j) {
				points.push_back(pts.at(j));
				pts.at(j).reference = element_vertex_count++;
			}
		}
	}

	for (auto& p : points)
	{
		point_coord_mean += p.point;
	}
	
	point_coord_mean /= static_cast<float>(points.size());

	for (auto& p : points)
	{
		p.point = p.point - point_coord_mean;
	}
}

void save_mesh_binary_ply(std::vector<marching_cubes::triangle> tris, std::vector<marching_cubes::vertex> points, int element_vertex_count, std::string const& filename)
{
	std::ofstream file(filename, std::ios::out | std::ios::binary);
	if (!file.is_open()) {
		std::cerr << "File " << filename << " can not be opened." << std::endl;
		throw std::logic_error("Could not open output file.");
	}

	std::cout << "Writing output to " << filename << "..." << std::endl;

	// Write header
	file << "ply" << std::endl;
	file << "format binary_little_endian 1.0" << std::endl;
	file << "element vertex " << element_vertex_count << std::endl;
	file << "property float x" << std::endl;
	file << "property float y" << std::endl;
	file << "property float z" << std::endl;
	file << "property float nx" << std::endl;
	file << "property float ny" << std::endl;
	file << "property float nz" << std::endl;
	file << "property uchar red" << std::endl;
	file << "property uchar green" << std::endl;
	file << "property uchar blue" << std::endl;
	file << "element face " << tris.size() << std::endl;
	file << "property list uchar int vertex_indices" << std::endl;
	file << "end_header" << std::endl;

	// Write points (removing duplicates)
	for (auto pt_it = points.begin(); pt_it != points.end(); ++pt_it) {

		for (int i = 0; i <= 2; ++i)
			file.write(reinterpret_cast<const char*> (&pt_it->point(i)), sizeof(pt_it->point(i)));
		for (int i = 0; i <= 2; ++i)
			file.write(reinterpret_cast<const char*> (&pt_it->normal(i)), sizeof(pt_it->normal(i)));
		int r = (int)pt_it->r, g = (int)pt_it->g, b = (int)pt_it->b;
		file.write(reinterpret_cast<const char*> (&r), sizeof(uchar));
		file.write(reinterpret_cast<const char*> (&g), sizeof(uchar));
		file.write(reinterpret_cast<const char*> (&b), sizeof(uchar));
	}

	// Write triangles
	int n = 3;
	for (auto tri_it = tris.begin(); tri_it != tris.end(); ++tri_it) {
		file.write(reinterpret_cast<const char*>(&n), sizeof(uchar));
		for (int i = 0; i <= 2; ++i) {
			file.write(reinterpret_cast<const char*> (&tri_it->points.at(i).reference), sizeof(tri_it->points.at(i).reference));
		}
	}

	file.close();
	std::cout << "Mesh successfully outputted." << std::endl;
}

void save_mesh_binary_ply( std::vector<marching_cubes::triangle> tris, std::string const& filename, bool allow_duplicates ) {
    // Open output file
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if(!file.is_open()) {
        std::cerr << "File " << filename << " can not be opened." << std::endl;
        throw std::logic_error("Could not open output file.");
    }

    std::vector<marching_cubes::vertex> points;
    int count = 0;
    if (!allow_duplicates) {
        std::cout << "Cleaning output (" << tris.size() * 3 << " vertices)" << std::endl;
        boost::progress_display progress(tris.size());
        // Get points without duplicates
        for (unsigned int i = 0; i < tris.size(); ++i) {
            std::vector<marching_cubes::vertex>& pts = tris.at(i).points;
            for (unsigned int j = 0; j < pts.size(); ++j) {
                marching_cubes::vertex& p = pts.at(j);
                auto where = std::find(points.begin(), points.end(), p); // Where this point may be found.
                if (where == points.end()) {
                    p.reference = count++;
                    points.push_back(p);
                } else {
                    // Mark as a duplicate in the triangle.
                    // Vector iterator is a random access iterator, so this is legal.
                    // Will not compile otherwise (std::distance would work, but its
                    // complexity increases on other containers...)
                    p.reference = where - points.begin();
                }
            }
            ++progress;
        }
    } else {
        for (unsigned int i = 0; i < tris.size(); ++i) {
            std::vector<marching_cubes::vertex>& pts = tris.at(i).points;
            for (unsigned int j = 0; j < pts.size(); ++j) {
                points.push_back(pts.at(j));
                pts.at(j).reference = count++;
            }
        }
    }

    std::cout << "Writing output to " << filename << "..." << std::endl;

    // Write header
    file << "ply" << std::endl;
    file << "format binary_little_endian 1.0" << std::endl;
    file << "element vertex " << count << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property float nx" << std::endl;
    file << "property float ny" << std::endl;
    file << "property float nz" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "element face " << tris.size() << std::endl;
    file << "property list uchar int vertex_indices" << std::endl;
    file << "end_header" << std::endl;

    // Write points (removing duplicates)
    for (auto pt_it = points.begin(); pt_it != points.end(); ++pt_it) {

        for (int i = 0; i <= 2; ++i)
            file.write( reinterpret_cast<const char*> (&pt_it->point(i)), sizeof(pt_it->point(i)) );
        for (int i = 0; i <= 2; ++i)
            file.write( reinterpret_cast<const char*> (&pt_it->normal(i)), sizeof(pt_it->normal(i)) );
        int r = (int)pt_it->r, g = (int)pt_it->g, b = (int)pt_it->b;
        file.write( reinterpret_cast<const char*> (&r), sizeof(uchar) );
        file.write( reinterpret_cast<const char*> (&g), sizeof(uchar) );
        file.write( reinterpret_cast<const char*> (&b), sizeof(uchar) );
    }

    // Write triangles
    int n = 3;
    for (auto tri_it = tris.begin(); tri_it != tris.end(); ++tri_it) {
        file.write( reinterpret_cast<const char*>(&n), sizeof(uchar) );
        for (int i = 0; i <= 2; ++i)
            file.write( reinterpret_cast<const char*> (&tri_it->points.at(i).reference), sizeof(tri_it->points.at(i).reference) );
    }

    file.close();
    std::cout << "Mesh successfully outputted." << std::endl;
}

void save_point_cloud_ply( std::vector<Eigen::Vector3f> pts, std::vector<Eigen::Vector3f> rgbs, std::string const& filename ) {
    // Open output file
    std::ofstream file(filename);
    if(!file.is_open()) {
        std::cerr << "File " << filename << " can not be opened." << std::endl;
        throw std::logic_error("Could not open output file.");
    }

    std::cout << "Writing output to " << filename << "..." << std::endl;

    // Write header
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << pts.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;


    for (int i = 0; i < static_cast<int>(pts.size()); ++i)
        file << pts[i].transpose() << " " << (255.f*rgbs[i]).cast<int>().transpose() << std::endl;

    file.close();
    std::cout << "Point cloud successfully outputted." << std::endl;
}

void save_point_cloud_binary_ply( std::vector<Eigen::Vector3f> pts, std::vector<Eigen::Vector3f> rgbs, std::string const& filename ) {
    // Open output file
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if(!file.is_open()) {
        std::cerr << "File " << filename << " can not be opened." << std::endl;
        throw std::logic_error("Could not open output file.");
    }

    std::cout << "Writing output to " << filename << "..." << std::endl;

    // Write header
    file << "ply" << std::endl;
    file << "format binary_little_endian 1.0" << std::endl;
    file << "element vertex " << pts.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        for (int j = 0; j <= 2; ++j)
            file.write( reinterpret_cast<const char*> (&pts[i](j)), sizeof(pts[i](j)) );
        Eigen::Vector3i rgb = (255.f*rgbs[i]).cast<int>();
        for (int j = 0; j <= 2; ++j)
            file.write( reinterpret_cast<const char*> (&rgb(j)), sizeof(uchar) );
    }

    file.close();
    std::cout << "Point cloud successfully outputted." << std::endl;
}
