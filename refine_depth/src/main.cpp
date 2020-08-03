//######################################################################
//#   Refine_depth Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <marker_detector.hpp>
#include <opencv/cxeigen.hpp>
#include "Eigen/Core"
#include <filesystem>
#include <random>
#include "io_utils.hpp"
#include "rgbd_types.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

namespace fs = std::filesystem;

using namespace Eigen;
using namespace std;
using namespace cv;

#define CHECK_PARAM_EXISTS(parser, param_name) \
     if (!parser.has(param_name)) {\
       cerr << "Missing parameter: " << param_name << endl;\
       return EXIT_FAILURE;\
     }

#define CHECK_VALID_FILE(f) \
     if (!fs::is_regular_file(f)) {\
       cerr << "Invalid file: " << f << endl;\
       return EXIT_FAILURE;\
     }

#define CHECK_VALID_DIR(dir) \
     if (!fs::is_directory(dir)) {\
       cerr << "Invalid directory: " << dir << endl;\
       return EXIT_FAILURE;\
     }


void draw_origin_axis(cv::Mat& colored_img, const cv::Mat& intrinsics, const cv::Mat& world_to_cam, double basis_length = 0.1)
{
	vector<Mat> p;
	p.push_back((cv::Mat_<double>(4, 1) << 0, 0, 0, 1));
	p.push_back((cv::Mat_<double>(4, 1) << basis_length, 0, 0, 1));
	p.push_back((cv::Mat_<double>(4, 1) << 0, basis_length, 0, 1));
	p.push_back((cv::Mat_<double>(4, 1) << 0, 0, basis_length, 1));

	// transform cube into camera reference frame
	vector<Mat> p_cam(p.size());
	for (size_t p_i = 0; p_i < p_cam.size(); ++p_i)
	{
		p_cam[p_i] = world_to_cam * p[p_i];
	}

	// Project points on the image
	vector<Mat> p2d(p.size());
	for (size_t p_i = 0; p_i < p2d.size(); ++p_i)
	{
		p2d[p_i] = intrinsics * p_cam[p_i].rowRange(0, 3);
		p2d[p_i] /= p2d[p_i].at<double>(2);
		//std::cout << p2d[p_i] << std::endl;
	}

	Point p0 = Point(p2d[0].at<double>(0), p2d[0].at<double>(1));
	Point px = Point(p2d[1].at<double>(0), p2d[1].at<double>(1));
	Point py = Point(p2d[2].at<double>(0), p2d[2].at<double>(1));
	Point pz = Point(p2d[3].at<double>(0), p2d[3].at<double>(1));

	line(colored_img, p0, px, cv::Scalar(0, 0, 255), 1, 8);
	line(colored_img, p0, py, cv::Scalar(0, 255, 0), 1, 8);
	line(colored_img, p0, pz, cv::Scalar(255, 0, 0), 1, 8);

	circle(colored_img, p0, 200 / 32.0, cv::Scalar(255, 255, 0), -1, 8);
}

void visualize_volume(const cv::Mat& img, const cv::Mat &intrinsics, const cv::Mat &rotation, const cv::Mat &translation,
                      const cv::Mat& llc, const cv::Mat& upc)
{
	cv::Mat drawing_image = img.clone();
	
	cv::Mat world_to_cam = cv::Mat::eye(4, 4, CV_64F);
	rotation.copyTo(world_to_cam.colRange(0, 3).rowRange(0, 3));
	translation.copyTo(world_to_cam.col(3).rowRange(0, 3));

	Point3f llc_p(llc);
	Point3f upc_p(upc);

	// Points in the world coordinates
	vector<Mat> p;
	p.push_back((cv::Mat_<double>(4, 1) << llc_p.x, llc_p.y, llc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << llc_p.x, llc_p.y, upc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << upc_p.x, llc_p.y, upc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << upc_p.x, llc_p.y, llc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << llc_p.x, upc_p.y, llc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << llc_p.x, upc_p.y, upc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << upc_p.x, upc_p.y, upc_p.z, 1));
	p.push_back((cv::Mat_<double>(4, 1) << upc_p.x, upc_p.y, llc_p.z, 1));

	// transform cube into camera reference frame
	vector<Mat> p_cam(p.size());
	for (size_t p_i = 0; p_i < p_cam.size(); ++p_i)
	{
		p_cam[p_i] = world_to_cam * p[p_i];
	}

	// Project points on the image
	vector<Mat> p2d(p.size());
	for (size_t p_i = 0; p_i < p2d.size(); ++p_i)
	{
		p2d[p_i] = intrinsics * p_cam[p_i].rowRange(0, 3);
		p2d[p_i] /= p2d[p_i].at<double>(2);
	}

	// Draw cube
	cv::Scalar line_color = cv::Scalar(0, 255, 255);
	// Generate edge points
	vector<pair<int, int>> edge_points;
	edge_points.push_back(make_pair(1, 5));
	edge_points.push_back(make_pair(2, 6));
	edge_points.push_back(make_pair(3, 7));
	edge_points.push_back(make_pair(0, 4));

	edge_points.push_back(make_pair(0, 1));
	edge_points.push_back(make_pair(1, 2));
	edge_points.push_back(make_pair(2, 3));
	edge_points.push_back(make_pair(3, 0));

	edge_points.push_back(make_pair(4, 7));
	edge_points.push_back(make_pair(7, 6));
	edge_points.push_back(make_pair(6, 5));
	edge_points.push_back(make_pair(5, 4));

	// Draw edges
	for (size_t e_i = 0; e_i < edge_points.size(); ++e_i)
	{
		int p0_i = edge_points[e_i].first;
		Point p0 = Point(p2d[p0_i].at<double>(0), p2d[p0_i].at<double>(1));
		int p1_i = edge_points[e_i].second;
		Point p1 = Point(p2d[p1_i].at<double>(0), p2d[p1_i].at<double>(1));
		line(drawing_image, p0, p1, line_color, 1, 8);
	}

	// Draw circles
	for (size_t p_i = 0; p_i < p2d.size(); ++p_i)
	{
		Point point = Point(p2d[p_i].at<double>(0), p2d[p_i].at<double>(1));
		circle(drawing_image, point, 200 / 32.0, line_color, -1, 8);
	}

	draw_origin_axis(drawing_image, intrinsics, world_to_cam);
	cv::imshow("image", drawing_image);
}


void visualize_volume(const cv::Mat& color_img, const Matrix3f& intrinsics, const Isometry3f& world_to_cam,
                      const std::vector<float>& volume)
{
	Mat intrinsics_cv = (Mat_<double>(3, 3) << intrinsics(0, 0), 0, intrinsics(0, 2), 0, intrinsics(1, 1), intrinsics(1, 2), 0, 0, 1);

	Mat llc = (Mat_<double>(3, 1) << volume[0], volume[2], volume[4]);
	Mat upc = (Mat_<double>(3, 1) << volume[1], volume[3], volume[5]);

	Mat rotation;
	Mat translation;
	Matrix4f pose_eigen = world_to_cam.matrix();
	
	Mat pose_cv;
	eigen2cv(pose_eigen, pose_cv);

	rotation = pose_cv(Range(0, 3), Range(0, 3));
	translation = pose_cv(Range(0, 3), Range(3, 4));

	visualize_volume(color_img, intrinsics_cv, rotation, translation, llc, upc);
}

struct Configuration
{
	std::string rgb_dir;
	std::string depth_dir;
	std::string board_file;
	std::string board_dict_file;
	Eigen::Matrix3f intrinsics;
	float marker_size;
	std::string out_dir;
};

bool parse_configuration(int argc, char** argv, Configuration& configuration)
{
	const String arg_keys =
		"{help h |      | help message   }"
		"{images_dir           |     | input images dir}"
		"{config_dir           |     | configuration dir with board.yml and dict.yml files         }"
		"{intrinsics_file           |     | intrinsics file}"
		"{marker_size           |     | marker size in meters}";

	cv::CommandLineParser parser(argc, argv, arg_keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		return false;
	}

	CHECK_PARAM_EXISTS(parser, "images_dir")
	string images_dir = parser.get<string>("images_dir");

	string rgb_dir = (fs::path(images_dir) / "rgb").string();
	string depth_dir = (fs::path(images_dir) / "depth").string();

	CHECK_VALID_DIR(rgb_dir)
	CHECK_VALID_DIR(depth_dir)

	CHECK_PARAM_EXISTS(parser, "config_dir")
	string config_dir = parser.get<string>("config_dir");

	string board_file = (fs::path(config_dir) / "board.yml").string();
	string board_dict_file = (fs::path(config_dir) / "dict.yml").string();
	CHECK_VALID_FILE(board_file)
	CHECK_VALID_FILE(board_dict_file)

	CHECK_PARAM_EXISTS(parser, "marker_size")
	float marker_size = parser.get<float>("marker_size");

	CHECK_PARAM_EXISTS(parser, "intrinsics_file")
	string intrinsics_file = parser.get<string>("intrinsics_file");
	Matrix3f intrinsics = read_intrinsics_from_file<float>(intrinsics_file);


	configuration.rgb_dir = rgb_dir;
	configuration.depth_dir = depth_dir;
	configuration.board_file = board_file;
	configuration.board_dict_file = board_dict_file;
	configuration.marker_size = marker_size;
	configuration.intrinsics = intrinsics;

	return true;
}


int main(int argc, char** argv)
{
	Configuration configuration;
	if (!parse_configuration(argc, argv, configuration)) return EXIT_FAILURE;

	vector<RgbdFile> rgbd_files = get_input_images(configuration.rgb_dir, configuration.depth_dir);

	MarkerDetector markers;
	markers.init(configuration.intrinsics);
	markers.loadDictionary(configuration.board_dict_file);
	markers.loadBoard(configuration.board_file);

	float marker_size = configuration.marker_size;

	std::vector<float> volume(6, 0.f);
	volume[0] = -3.f * marker_size;
	volume[1] = 3.f * marker_size;
	volume[2] = -3.f * marker_size;
	volume[3] = 3.f * marker_size;
	volume[4] = -0.3f;
	volume[5] = 0.0f;
	// volume[4] = 0.0f; volume[5] = 0.3f;

	vector<float> gt_depth_values;
	vector<float> measured_depth_values;

	for (const auto& rgbd_file : rgbd_files)
	{
		RgbdFrame rgbd_frame = get_rgbd_frame(rgbd_file);

		const Mat& color = rgbd_frame.first;
		const Mat& depth = rgbd_frame.first;

		vector<cv::Point2f> points2d;
		vector<cv::Point3f> points3d;

		if (markers.detect(color, marker_size, points2d, points3d) > 0)
		{
			cerr << "Failed to detect the markers for " << rgbd_frame.first << endl;
			continue;
		}

		if (markers.getNumDetected() < 8)
		{
			cerr << "Insufficient number of markerks detected: " << markers.getNumDetected() << " for " <<
				rgbd_frame.first << endl;
			continue;
		}

		const Isometry3f &pose = markers.board.pose;
		Isometry3f world_to_cam = pose.inverse();

		visualize_volume(color, configuration.intrinsics, world_to_cam, volume);
		int key = cv::waitKey(10);

		if (key != 's' && key != 'S')
		{
			for (int i = 0; i < static_cast<int>(points2d.size()); i++)
			{
				cv::Point2f p2d = points2d[i];

				int x = static_cast<int>(round(p2d.x));
				int y = static_cast<int>(round(p2d.y));

				float measured_depth = depth.at<float>(y, x);

				if (isfinite(measured_depth) && measured_depth)
				{
					Point3f p3d = points3d[i];
					Vector3f point3d = {p3d.x, p3d.y, p3d.z};

					Vector3f point3d_in_camera_coord = world_to_cam * point3d;
					float gt_depth = point3d_in_camera_coord[2];

					gt_depth_values.push_back(gt_depth);
					measured_depth_values.push_back(measured_depth);
				}
			}
		}
		else
		{
			cout << "Skipped frame " << rgbd_file.first << endl;
		}
	}

	int data_size = static_cast<int>(measured_depth_values.size());
	int train_data_size = static_cast<int>(0.9 * data_size);
	int test_data_size = data_size - train_data_size;

	cout << "Total dataset size: " << data_size << endl;

	MatrixXf train_data_mat(train_data_size, 2);
	VectorXf train_gt_vec(train_data_size);

	MatrixXf test_data_mat(test_data_size, 2);
	VectorXf test_gt_vec(test_data_size);

	float lambda = 0.01f;

	vector<int> data_indices(data_size);

	for (int i = 0; i < data_size; i++) data_indices[i] = i;

	std::random_device rng;
	std::mt19937 urng(rng());
	std::shuffle(data_indices.begin(), data_indices.end(), urng);

	for (int i = 0; i < data_size; i++)
	{
		float measured = measured_depth_values[i];
		float gt = gt_depth_values[i];

		if (i < train_data_size)
		{
			train_data_mat(i, 0) = measured;
			train_data_mat(i, 1) = 1.0f;

			train_gt_vec(i) = gt;
		}
		else
		{
			int test_idx = i - train_data_size;

			test_data_mat(test_idx, 0) = measured;
			test_data_mat(test_idx, 1) = 1.0f;

			test_gt_vec(test_idx) = gt;
		}
	}

	MatrixXf train_data_mat_t = train_data_mat.transpose();
	Vector2f params = (train_data_mat_t * train_data_mat + lambda * Matrix2f::Identity()).inverse() * (train_data_mat_t
		* train_gt_vec);

	float avg_error_train = ((train_data_mat * params) - train_gt_vec).norm() / static_cast<float>(train_data_size);
	float avg_error_test = ((test_data_mat * params) - test_gt_vec).norm() / static_cast<float>(test_data_size);

	float mean_abs_diff_train = 0.0f;
	float mean_abs_diff_test = 0.0f;
	float mean_abs_diff_test_before = 0.0;


	for (int i = 0; i < train_data_size; i++)
	{
		mean_abs_diff_train += abs(train_data_mat(i, 0) - train_gt_vec(i));
	}

	mean_abs_diff_train /= static_cast<float>(train_data_size);

	VectorXf test_refined = test_data_mat * params;

	for (int i = 0; i < test_data_size; i++)
	{
		mean_abs_diff_test_before += abs(test_data_mat(i, 0) - test_gt_vec(i));
		mean_abs_diff_test += abs(test_refined(i) - test_gt_vec(i));
	}

	mean_abs_diff_test_before /= static_cast<float>(test_data_size);
	mean_abs_diff_test /= static_cast<float>(test_data_size);

	VectorXf test_data_vec = test_data_mat.block(0, 0, test_data_size, 1);
	float test_error_before = (test_data_vec - test_gt_vec).norm() / static_cast<float>(test_data_size);

	cout << "Test error before refinement" << test_error_before << endl;

	cout << "Estimated params: " << params << endl;
	cout << "Train avg error: " << avg_error_train << endl;
	cout << "Test avg error: " << avg_error_test << endl;

	cout << "Mean abs diff train" << mean_abs_diff_train << endl;
	cout << "Mean abs diff test before: " << mean_abs_diff_test_before << endl;
	cout << "Mean abs diff test: " << mean_abs_diff_test << endl;

	return 0;
}
