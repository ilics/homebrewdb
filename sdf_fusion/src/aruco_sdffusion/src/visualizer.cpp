//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include "visualizer.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace Eigen;
using namespace std;

vector<Mat> transform_and_project(const vector<Mat>& points, const Mat& intrinsics, const Mat& pose)
{
	vector<Mat> projected_2d_points(points.size());
	transform(points.begin(), points.end(), projected_2d_points.begin(), [&pose, &intrinsics] (const auto &point)
	{
		Mat transformed = pose * point;
		Mat projected = intrinsics * transformed.rowRange(0, 3);
		projected /= projected.at<float>(2);
		return projected;
	});

	return projected_2d_points;
}


void draw_origin_axis(Mat &colored_img, const Mat& intrinsics, const Mat& pose, float basis_length = 0.1) {
	vector<Mat> axes_endpoints;
	axes_endpoints.push_back((Mat_<float>(4, 1) << 0, 0, 0, 1));
	axes_endpoints.push_back((Mat_<float>(4, 1) << basis_length, 0, 0, 1));
	axes_endpoints.push_back((Mat_<float>(4, 1) << 0, basis_length, 0, 1));
	axes_endpoints.push_back((Mat_<float>(4, 1) << 0, 0, basis_length, 1));

	// transform cube into camera reference frame

	vector<Mat> p2d = transform_and_project(axes_endpoints, intrinsics, pose);

	Point p0 = Point(p2d[0].at<float>(0), p2d[0].at<float>(1));
	Point px = Point(p2d[1].at<float>(0), p2d[1].at<float>(1));
	Point py = Point(p2d[2].at<float>(0), p2d[2].at<float>(1));
	Point pz = Point(p2d[3].at<float>(0), p2d[3].at<float>(1));

	line(colored_img, p0, px, cv::Scalar(0, 0, 255), 1, 8);
	line(colored_img, p0, py, cv::Scalar(0, 255, 0), 1, 8);
	line(colored_img, p0, pz, cv::Scalar(255, 0, 0), 1, 8);

	circle(colored_img, p0, 200 / 32.0, cv::Scalar(255, 255, 0), -1, 8);
}


void visualize_volume(const Mat &source_image, const Mat &intrinsics, const Mat &rotation, const Mat &translation, const Mat& llc, const Mat& upc)
{
	Mat image = source_image.clone();
	
	Mat pose = Mat::eye(4, 4, CV_32F);
	rotation.copyTo(pose.colRange(0, 3).rowRange(0, 3));
	translation.copyTo(pose.col(3).rowRange(0, 3));

	Point3f llc_p(llc);
	Point3f upc_p(upc);

	// Points in the world coordinates
	vector<Mat> volume_points;
	volume_points.push_back((Mat_<float>(4, 1) << llc_p.x, llc_p.y, llc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << llc_p.x, llc_p.y, upc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << upc_p.x, llc_p.y, upc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << upc_p.x, llc_p.y, llc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << llc_p.x, upc_p.y, llc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << llc_p.x, upc_p.y, upc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << upc_p.x, upc_p.y, upc_p.z, 1));
	volume_points.push_back((Mat_<float>(4, 1) << upc_p.x, upc_p.y, llc_p.z, 1));

	// transform cube into camera reference frame
	vector<Mat> p2d = transform_and_project(volume_points, intrinsics, pose);

	// Draw cube
	Scalar line_color = Scalar(148, 24, 248);
	// Generate edge points
	vector<pair<int, int>> edge_point_indices;
	edge_point_indices.push_back(make_pair(1, 5));
	edge_point_indices.push_back(make_pair(2, 6));
	edge_point_indices.push_back(make_pair(3, 7));
	edge_point_indices.push_back(make_pair(0, 4));

	edge_point_indices.push_back(make_pair(0, 1));
	edge_point_indices.push_back(make_pair(1, 2));
	edge_point_indices.push_back(make_pair(2, 3));
	edge_point_indices.push_back(make_pair(3, 0));

	edge_point_indices.push_back(make_pair(4, 7));
	edge_point_indices.push_back(make_pair(7, 6));
	edge_point_indices.push_back(make_pair(6, 5));
	edge_point_indices.push_back(make_pair(5, 4));

	// Draw edges

	for (const auto &point_indices : edge_point_indices)
	{
		int p0_i = point_indices.first;
		Point p0 = Point(p2d[p0_i].at<float>(0), p2d[p0_i].at<float>(1));
		int p1_i = point_indices.second;
		Point p1 = Point(p2d[p1_i].at<float>(0), p2d[p1_i].at<float>(1));
		line(image, p0, p1, line_color, 1, 8);
		
	}

	// Draw circles
	for (const auto& point : p2d)
	{
		Point point_to_draw = Point(point.at<float>(0), point.at<float>(1));
		circle(image, point_to_draw, 200 / 32.0, line_color, -1, 8);
	}

	draw_origin_axis(image, intrinsics, pose);

	Mat image_to_show;

	// resize if too big for showing
	if (image.cols > 800)
	{
		Size show_size = Size(image.cols / 2, image.rows / 2);
		resize(image, image_to_show, show_size);
	} else
	{
		image_to_show = image;
	}
	
	
	imshow("image", image_to_show);
}


void visualize_volume(const Mat& input_image, const Matrix3f& intrinsics, const Isometry3f& pose, const vector<float>& volume)
{
	Mat intrinsics_cv;

	eigen2cv(intrinsics, intrinsics_cv);

	Mat llc = (Mat_<float>(3, 1) << volume[0], volume[2], volume[4]);
	Mat upc = (Mat_<float>(3, 1) << volume[1], volume[3], volume[5]);

	const Matrix4f &pose_eigen = pose.matrix();
	Mat pose_cv;
	eigen2cv(pose_eigen, pose_cv);

	Mat rotation = pose_cv(Range(0, 3), Range(0, 3));
	Mat translation = pose_cv(Range(0, 3), Range(3, 4));

	visualize_volume(input_image, intrinsics_cv, rotation, translation, llc, upc);
}