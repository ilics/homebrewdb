//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include <opencv/cv.hpp>
#include "optimizer.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "util.h"
#include <Eigen/Dense>
#include "correspondence_finder.h"
#include "frame_payload.h"
#include <utility>

#define GLOG_NO_ABBREVIATED_SEVERITIES

using namespace ceres;
using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct ModelPoseFunctor {
	ModelPoseFunctor(Eigen::Vector2d pixel_point,  Eigen::Vector3d world_point, Eigen::Matrix3d intrinsics,
		double* scene_inverse_angle_axis, double* scene_pose_inverse_translation, Eigen::Vector2d rgb_edge_normal, double std_dev)
		:pixel_point(std::move(pixel_point)), world_point(std::move(world_point)), intrinsics(std::move(intrinsics)),
		scene_inverse_angle_axis(scene_inverse_angle_axis), scene_pose_inverse_translation(scene_pose_inverse_translation),
		rgb_edge_normal(std::move(rgb_edge_normal)), std_dev(std_dev) {}

	template <typename T>  bool operator()(const T* const rotation_to_optimize, const T* const translation_to_optimize, T* residual) const {

		T p[3];

		p[0] = T(world_point[0]);
		p[1] = T(world_point[1]);
		p[2] = T(world_point[2]);

		ceres::AngleAxisRotatePoint(rotation_to_optimize, p, p);

		p[0] += translation_to_optimize[0];
		p[1] += translation_to_optimize[1];
		p[2] += translation_to_optimize[2];

		T scene_inverse_angle_axis_t[3];
		scene_inverse_angle_axis_t[0] = T(scene_inverse_angle_axis[0]);
		scene_inverse_angle_axis_t[1] = T(scene_inverse_angle_axis[1]);
		scene_inverse_angle_axis_t[2] = T(scene_inverse_angle_axis[2]);

		ceres::AngleAxisRotatePoint(scene_inverse_angle_axis_t, p, p);

		T scene_inverse_translation_t[3];
		scene_inverse_translation_t[0] = T(scene_pose_inverse_translation[0]);
		scene_inverse_translation_t[1] = T(scene_pose_inverse_translation[1]);
		scene_inverse_translation_t[2] = T(scene_pose_inverse_translation[2]);

		p[0] += scene_inverse_translation_t[0];
		p[1] += scene_inverse_translation_t[1];
		p[2] += scene_inverse_translation_t[2];

		T predicted_x = ((T(intrinsics(0, 0))*p[0]) / p[2]) + T(intrinsics(0, 2));
		T predicted_y = ((T(intrinsics(1, 1))*p[1]) / p[2]) + T(intrinsics(1, 2));

		residual[0] = ((predicted_x - T(pixel_point[0])) * T(rgb_edge_normal[0]) + 
			(predicted_y - T(pixel_point[1])) * T(rgb_edge_normal[1])) / T(std_dev);
		return true;
	}

	Eigen::Vector2d pixel_point;
	Eigen::Vector3d world_point;

	Eigen::Matrix3d intrinsics;
	double* scene_inverse_angle_axis;
	double* scene_pose_inverse_translation;
	Eigen::Vector2d rgb_edge_normal;
	double std_dev;
};


Eigen::Matrix4d optimize_model_pose(const vector<FramePayload> &frame_payloads, const Eigen::Matrix4d &model_pose, const Eigen::Matrix3d &intrinsics, double residuals_std_dev)
{
	double* angle_axis_to_optimize = new double[3];
	double* translation_to_optimize = new double[3];

	Eigen::Matrix3d model_rotation = model_pose.block<3, 3>(0, 0);
	Eigen::Vector3d model_translation = model_pose.block<3, 1>(0, 3);

	ceres::RotationMatrixToAngleAxis(model_rotation.data(), angle_axis_to_optimize);
	memcpy(translation_to_optimize, model_translation.data(), 3 * sizeof(double));

	std::cout << "angle axis : " << angle_axis_to_optimize[0]
		<< " -> " << angle_axis_to_optimize[1]
		<< " -> " << angle_axis_to_optimize[2]
		<< ", translation " << translation_to_optimize[0]
		<< " -> " << translation_to_optimize[1]
		<< " -> " << translation_to_optimize[2] << "\n";

	Problem problem;

	vector<double*> angle_axis_scene_invere_rotations;
	vector<double*> scene_invere_translations;
	cout << "Residuals std dev: " << residuals_std_dev << endl;

	for (const FramePayload &payload : frame_payloads)
	{
		const Correspondence &correspondence = payload.correspondence;

		Eigen::Matrix4d scene_pose_inverse = payload.scene_pose.inverse();

		Eigen::Matrix3d scene_pose_inverse_rotation = scene_pose_inverse.block<3, 3>(0, 0);
		Eigen::Vector3d scene_pose_inverse_translation = scene_pose_inverse.block<3, 1>(0, 3);

		double* angle_axis_scene_inverse = new double[3];
		ceres::RotationMatrixToAngleAxis(scene_pose_inverse_rotation.data(), angle_axis_scene_inverse);

		double* scene_pose_inverse_translation_array = new double[3];
		memcpy(scene_pose_inverse_translation_array, scene_pose_inverse_translation.data(), 3 * sizeof(double));

		angle_axis_scene_invere_rotations.push_back(angle_axis_scene_inverse);
		scene_invere_translations.push_back(scene_pose_inverse_translation_array);

		for (int i = 0; i < correspondence.get_number_of_correspondences(); i++) {
			auto pixel_point = correspondence.corresponding_points[i].cast<double>();
			auto world_point = correspondence.world_points[i].cast<double>();

			Eigen::Vector2d rgb_edge_normal = correspondence.get_rgb_normal(i).cast<double>();

			CostFunction *cost_function =
				new AutoDiffCostFunction<ModelPoseFunctor, 1, 3, 3>(new ModelPoseFunctor(pixel_point, world_point, intrinsics,
					angle_axis_scene_inverse, scene_pose_inverse_translation_array,
					rgb_edge_normal, residuals_std_dev));

			TukeyLoss* loss_function = new TukeyLoss(4.365);
			problem.AddResidualBlock(cost_function, loss_function, angle_axis_to_optimize, translation_to_optimize);
		}
	}

	// run the solver
	Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.max_num_iterations = 100;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";

	std::cout << "optimized angle axis : " << angle_axis_to_optimize[0]
		<< " -> " << angle_axis_to_optimize[1]
		<< " -> " << angle_axis_to_optimize[2]
		<< ", optimized translation " << translation_to_optimize[0]
		<< " -> " << translation_to_optimize[1]
		<< " -> " << translation_to_optimize[2] << "\n";

	double optimal_rotation_matrix_array[9];
	ceres::AngleAxisToRotationMatrix(angle_axis_to_optimize, optimal_rotation_matrix_array);

	Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(optimal_rotation_matrix_array);
	Eigen::Vector3d translation_vector = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(translation_to_optimize);

	delete[] angle_axis_to_optimize;
	delete[] translation_to_optimize;

	for (auto &arr : angle_axis_scene_invere_rotations) {
		delete[] arr;
	}

	for (auto &arr : scene_invere_translations) {
		delete[] arr;
	}

	return create_transformation(rotation_matrix, translation_vector);
}
