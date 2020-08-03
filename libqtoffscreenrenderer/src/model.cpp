//######################################################################
//#   Liboffscreenrenderer Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>

#include <opencv2/viz.hpp>

#include "model.h"
#include "painter.h"

namespace fs = std::filesystem;
using namespace std;
using namespace cv;
using namespace Eigen;

#define USE_VBO



Model::Model(): m_cube_size(0.005)
{
}

Model::Model(float cube_radius): m_cube_size(cube_radius)
{
}

Model::~Model()
{
}


void Model::paint()
{
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	if (!m_points.empty())
	{
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
#ifdef USE_VBO
		if (!m_faces.empty()) Painter::getSingleton()->drawVBOs(m_vbo_vertices, m_vbo_indices, m_faces.size() * 3);
		else Painter::getSingleton()->drawVBOs(m_vbo_vertices, 0, m_points.size());
#else
        glVertexPointer(3,GL_FLOAT,2*sizeof(Vector3f),&(vertex_data[0]));
        glColorPointer (3,GL_FLOAT,2*sizeof(Vector3f),&(vertex_data[1]));
        if(!faces.empty()) glDrawElements(GL_TRIANGLES,faces.size()*3,GL_UNSIGNED_INT,faces.data());
        else glDrawArrays(GL_POINTS,0,points.size());
#endif
		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
	}
}



void Model::bindVBOs()
{
	// Interleaving vertex and color data for faster rendering
	m_vertex_data.resize(m_points.size() * 2);
	for (uint i = 0; i < m_points.size(); ++i)
	{
		m_vertex_data[i * 2 + 0] = m_points[i];
		m_vertex_data[i * 2 + 1] = m_colors[i];
		//m_vertex_data[i*2+1] = m_localCoordColors[i];
	}
#ifdef USE_VBO  // Bind VBO data onto GPU
	Painter::getSingleton()->bindVBOs(m_vertex_data, m_faces, m_vbo_vertices, m_vbo_indices);
#endif
}


float Model::computeMeshResolution()
{
	assert(!m_faces.empty());
	float meshResolution = 0;
	for (Vector3i& tri : m_faces)
	{
		meshResolution += (m_points[tri(0)] - m_points[tri(1)]).norm();
		meshResolution += (m_points[tri(1)] - m_points[tri(2)]).norm();
		meshResolution += (m_points[tri(2)] - m_points[tri(0)]).norm();
	}
	return meshResolution / (m_faces.size() * 3.0f);
}


void Model::computeLocalCoordsColors()
{
	assert(!m_colors.empty());
	m_localCoordColors.clear();
	for (const auto& p : m_points)
	{
		Vector3f col = (p - bb_min).array() / bb_max.array();
		col = Vector3f(0.5, 0.5, 0.5f) + (col - Vector3f(0.5, 0.5, 0.5f)) * 0.7f;
		m_localCoordColors.push_back(col);
	}
}


vector<bool> Model::computeEdgePoints()
{
	assert(!m_faces.empty());

	vector<bool> list(m_points.size(), false);

	// Extract all edges and vertex2face info
	vector<Vector2i> edges;
	vector<vector<int>> vert2face(m_points.size());
	for (uint i = 0; i < m_faces.size(); ++i)
	{
		const Vector3i& tri = m_faces[i];
		edges.push_back(Vector2i(tri(0), tri(1)));
		edges.push_back(Vector2i(tri(1), tri(2)));
		edges.push_back(Vector2i(tri(2), tri(0)));
		vert2face[tri(0)].push_back(i);
		vert2face[tri(1)].push_back(i);
		vert2face[tri(2)].push_back(i);
	}

	for (Vector2i& e : edges)
	{
		// For each edge, count if it is connected to more than 1 face
		int counter = 0;
		for (int i : vert2face[e(0)])
		{
			Vector3i& tri = m_faces[i];
			if (tri(0) == e(1) || tri(1) == e(1) || tri(2) == e(1))
				counter++;
		}
		list[e(0)] = list[e(0)] | (counter < 2);
		list[e(1)] = list[e(1)] | (counter < 2);
	}

	return list;
}


void Model::computeVertexNormals()
{
	assert(!m_faces.empty());

	// Extract vertex2face info
	m_normals.clear();
	vector<vector<int>> vert2face(m_points.size());
	for (uint i = 0; i < m_faces.size(); ++i)
	{
		vert2face[m_faces[i](0)].push_back(i);
		vert2face[m_faces[i](1)].push_back(i);
		vert2face[m_faces[i](2)].push_back(i);
	}

	// Build average vertex normal over all adjacent faces
	for (uint i = 0; i < m_points.size(); ++i)
	{
		Vector3f normal(0, 0, 0);
		for (int v : vert2face[i])
		{
			Vector3i& f = m_faces[v];
			normal += (m_points[f(1)] - m_points[f(0)]).cross(m_points[f(2)] - m_points[f(0)]);
		}
		assert(normal.allFinite());
		m_normals.push_back(normal.normalized());
	}
}



void Model::computeBoundingBox()
{
	float val = numeric_limits<float>::max();
	bb_min << val, val, val;
	bb_max = -bb_min;
	for (Vector3f& p : m_points)
	{
		for (int k = 0; k < 3; ++k) bb_min(k) = min(bb_min(k), p(k));
		for (int k = 0; k < 3; ++k) bb_max(k) = max(bb_max(k), p(k));
	}
	boundingBox.col(0) << bb_min(0), bb_min(1), bb_min(2);
	boundingBox.col(1) << bb_min(0), bb_max(1), bb_min(2);
	boundingBox.col(2) << bb_max(0), bb_max(1), bb_min(2);
	boundingBox.col(3) << bb_max(0), bb_min(1), bb_min(2);
	boundingBox.col(4) << bb_min(0), bb_min(1), bb_max(2);
	boundingBox.col(5) << bb_min(0), bb_max(1), bb_max(2);
	boundingBox.col(6) << bb_max(0), bb_max(1), bb_max(2);
	boundingBox.col(7) << bb_max(0), bb_min(1), bb_max(2);
}


void Model::subsampleCloud(float voxel_size)
{
	assert(m_points.size() == m_normals.size());

	Vector3f extend = (bb_max - bb_min).cwiseAbs();
	Vector3i res = Vector3i(1, 1, 1) + (extend / voxel_size).cast<int>();

	vector<bool> occupancy(res(0) * res(1) * res(2), false);
	vector<Vector3f> normals(res(0) * res(1) * res(2), Vector3f(0, 0, 0));

	for (uint i = 0; i < m_points.size(); ++i)
	{
		Vector3i vox = ((m_points[i] - bb_min) / voxel_size).cast<int>();
		int index = vox(2) * res(1) * res(0) + vox(1) * res(0) + vox(0);
		occupancy[index] = true;
		normals[index] += m_normals[i];
	}

	m_subpoints.clear();
	m_subnormals.clear();
	m_subcolors.clear();
	for (int z = 0; z < res(2); ++z)
		for (int y = 0; y < res(1); ++y)
			for (int x = 0; x < res(0); ++x)
			{
				int index = z * res(1) * res(0) + y * res(0) + x;
				if (occupancy[index])
				{
					m_subpoints.push_back(Vector3f(x + 0.5f, y + 0.5f, z + 0.5f) * voxel_size + bb_min);
					m_subnormals.push_back(normals[index].normalized());
					Vector3f col = m_colors[index] * 255.f;
					m_subcolors.push_back(Vec3b(col(0), col(1), col(2)));
				}
			}

#if 0
    cv::viz::Viz3d show;
    cv::Mat points(1,m_points.size(),CV_32FC3, m_points.data());
    cv::Mat sub(1,m_subpoints.size(),CV_32FC3, m_subpoints.data());
    cv::Mat nors(1,m_subnormals.size(),CV_32FC3, m_subnormals.data());
    cv::Mat cols(1,m_subcolors.size(),CV_8UC3, m_subcolors.data());
    //show.showWidget("original",viz::WCloud(points,cv::viz::Color::green()));
    show.showWidget("sub",viz::WCloud(sub,cols));
    show.showWidget("normals",viz::WCloudNormals(sub,nors,1,0.01));
    show.spin();
#endif
}


void Model::savePLY(string filename)
{
	ofstream file(filename, ios::out | ios::binary);
	file << "ply" << endl;
	file << "format binary_little_endian 1.0" << endl;
	file << "element vertex " << m_points.size() << endl;
	file << "property float x" << endl;
	file << "property float y" << endl;
	file << "property float z" << endl;
	if (!m_normals.empty())
	{
		file << "property float nx" << endl;
		file << "property float ny" << endl;
		file << "property float nz" << endl;
	}
	if (!m_colors.empty())
	{
		file << "property uchar red" << endl;
		file << "property uchar green" << endl;
		file << "property uchar blue" << endl;
	}
	if (!m_faces.empty())
	{
		file << "element face " << m_faces.size() << endl;
		file << "property list uchar int vertex_indices" << endl;
	}
	file << "end_header" << endl;

	for (uint i = 0; i < m_points.size(); i++)
	{
		file.write((const char*)(&m_points[i]), 3 * sizeof(float));
		if (!m_normals.empty())
			file.write((const char*)(&m_normals[i]), 3 * sizeof(float));
		if (!m_colors.empty())
		{
			Vec3b col(255 * m_colors[i](0), 255 * m_colors[i](1), 255 * m_colors[i](2));
			file.write((const char*)(&col), 3 * sizeof(uchar));
		}
	}
	
	for (uint i = 0; i < m_faces.size(); i++)
	{
		const char tmp = 3;
		file.write(&tmp, sizeof(char));
		file.write((const char*)(&m_faces[i]), 3 * sizeof(int));
	}
	file.close();
}


bool Model::loadPLY(string filename)
{
	if (!fs::is_regular_file(filename))
	{
		cerr << filename << " does not exist" << endl;
		return false;
	}


	cv::viz::Mesh mesh = cv::viz::Mesh::load(filename);

	m_points.resize(mesh.cloud.cols);
	// cout << CV_ELEM_SIZE1(DataType<float>::depth) << endl;
	// cout << mesh.cloud.at<float>(0, 3) << endl;
	for (int i = 0; i < mesh.cloud.cols; ++i)
		// m_points[i] = mesh.cloud.at<Vector3f>(0,i);
	{
		// m_points[i] = mesh.cloud.at<Vector3f>(0,i);
		Vec3f& src = mesh.cloud.at<Vec3f>(0, i);
		m_points[i] = Vector3f(src[0], src[1], src[2]);
	}

	m_faces.clear();
	for (int i = 0; i < mesh.polygons.cols / 4; i++)
	{
		Vec4i& tri = mesh.polygons.at<Vec4i>(0, i);
		// Vector4i &tri = mesh.polygons.at<Vector4i>(0,i);
		assert(tri(0)==3); // Assert that all faces are triangles
		m_faces.push_back(Vector3i(tri(1), tri(2), tri(3)));
	}

	m_colors.clear();
	if (mesh.colors.empty())
	{
		cerr << "Model - no colors in file" << endl;
		m_colors.assign(m_points.size(), Vector3f(127, 127, 127));
	}
	else
		for (int i = 0; i < mesh.colors.cols; ++i)
		{
			const Vec3b& col = mesh.colors.at<Vec3b>(0, i);
			m_colors.push_back(Vector3f(col[0], col[1], col[2]));
		}

	assert(!m_points.empty());

	centroid.setZero();
	for (const auto& p : m_points) centroid += p;
	centroid /= m_points.size();

	// Process colors
	for (Vector3f& c : m_colors) c /= 255.0f; // Normalized color for opengl


	computeBoundingBox();
	computeVertexNormals();
	computeLocalCoordsColors();
	bindVBOs();
	//subsampleCloud(m_cube_size);
	m_diameter = (bb_max - bb_min).norm();
	return true;
}

