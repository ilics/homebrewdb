//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include <Eigen/Core>
#include "utils.hpp"
#include "io_utils.hpp"

#include "reconstructor_3d.hpp"
#include "marching_cubes.hpp"
#include "rgbd_types.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

sdf initialize_sdf(const Configuration &configuration, const vector<float> &volume)
{
	float voxel_size = configuration.get_voxel_size();

	int voxel_count_x = static_cast<int>(std::ceil((volume[1] - volume[0]) / voxel_size));
	int voxel_count_y = static_cast<int>(std::ceil((volume[3] - volume[2]) / voxel_size));
	int voxel_count_z = static_cast<int>(std::ceil((volume[5] - volume[4]) / voxel_size));

	sdf model(voxel_count_x, voxel_count_y, voxel_count_z, voxel_size, 0.f, 1.f);

	return model;
}

template <typename T>
Transform<T, 3, Isometry> to_isometry(const Matrix<T, 4, 4> &pose_matrix)
{
	Transform<T, 3, Isometry> pose;
	pose.setIdentity();

	pose.linear() = pose_matrix.block<3, 3>(0, 0);
	pose.translation() = pose_matrix.block<3, 1>(0, 3);
	
	return pose;
}

sdf generate_tsdf(const Mat& depth_map, const Mat& color_map, const Isometry3f &transformation_matrix, const Matrix3f &intrinsics, const vector<float>& dimensions,
	const Configuration& configuration) {

	sdf s = initialize_sdf(configuration, dimensions);
	const float voxel_size = configuration.get_voxel_size();

	float delta = voxel_size;
	float eta = 3 * voxel_size;

	Vector3f lower_left(dimensions[0], dimensions[2], dimensions[4]);

#pragma omp parallel for
	for (int z = 0; z < s.size_z; ++z)
		for (int y = 0; y < s.size_y; ++y)
			for (int x = 0; x < s.size_x; ++x) {
				Vector3f rp = lower_left + voxel_size * Vector3f(x + 0.5f, y + 0.5f, z + 0.5f);
				Vector3f transformed_point = transformation_matrix * rp;

				Vector2f point2d = project_point(transformed_point, intrinsics);

				int imgx = static_cast<int>(ceil(point2d(0)));
				int imgy = static_cast<int>(ceil(point2d(1)));

				if (imgx >= 0 && imgy >= 0 && imgx < depth_map.cols && imgy < depth_map.rows) {

					float depth = depth_map.at<float>(imgy, imgx);
					if (isfinite(depth) && depth > .0f) {
						const float signed_distance = depth - transformed_point(2);
						s.weights.at(x, y, z) = signed_distance > -eta;
						s.distance_field.at(x, y, z) = min(1.f, max(-1.f, signed_distance / delta));

						cv::Vec3b color = color_map.at<cv::Vec3b>(imgy, imgx);
						s.color_field.red.at(x, y, z) = color[2] / 255.f;
						s.color_field.green.at(x, y, z) = color[1] / 255.f;
						s.color_field.blue.at(x, y, z) = color[0] / 255.f;
					} // otherwise no need to change values (works only when they are initialized)
				}
			}

	return s;
}

void integrate_into_weighted_average(sdf& wa, const sdf& s) {
	color_cube<float> & wa_colors = wa.color_field;
	const color_cube<float> & s_colors = s.color_field;

#pragma omp parallel for
	for (int z = 0; z < wa.size_z; ++z)
		for (int y = 0; y < wa.size_y; ++y)
			for (int x = 0; x < wa.size_x; ++x) {
				float old_weight = wa.weights.at(x, y, z);
				float added_weight = s.weights.at(x, y, z);
				float new_weight = old_weight + added_weight;

				if (new_weight > 0.f) {
					//wa.distance_field.at(x, y, z) /= wa.weights.at(x, y, z);
					wa.distance_field.at(x, y, z) = (wa.distance_field.at(x, y, z) * old_weight + s.distance_field.at(x, y, z) * added_weight) / new_weight;

					//if (fabs(wa.distance_field.at(x, y, z)) < 1.f) { // assign color only at near-surface voxels
					wa_colors.red.at(x, y, z) = (wa_colors.red.at(x, y, z)   * old_weight + s_colors.red.at(x, y, z)   * added_weight) / new_weight;
					wa_colors.green.at(x, y, z) = (wa_colors.green.at(x, y, z) * old_weight + s_colors.green.at(x, y, z) * added_weight) / new_weight;
					wa_colors.blue.at(x, y, z) = (wa_colors.blue.at(x, y, z)  * old_weight + s_colors.blue.at(x, y, z)  * added_weight) / new_weight;
					//}
				}
				else {
					wa.distance_field.at(x, y, z) = -1.f;
					// and do not change color
				}
				wa.weights.at(x, y, z) = new_weight;
			}
}

sdf reconstruct_sdf(const Configuration& configuration, const vector<float>& volume, const vector<RgbdFile> &input_images, const vector<Matrix4f> &poses)
{

	sdf model = initialize_sdf(configuration, volume);
	const int number_of_frames = static_cast<int>(input_images.size());

	int64 elapsed_time = 0;

	for (int i = 0; i < number_of_frames; ++i)
	{
		RgbdFrame rgbd_frame = get_rgbd_frame(input_images[i]);
		Isometry3f pose = to_isometry(poses[i]);

		auto start_time = getTickCount();
		sdf current_sdf = generate_tsdf(rgbd_frame.second, rgbd_frame.first, pose.inverse(), configuration.get_intrinsics(),
		                                   volume, configuration);
		integrate_into_weighted_average(model, current_sdf);
		elapsed_time += getTickCount() - start_time;
		cout << "Integrated " << i + 1 << " out of " << number_of_frames << " frames into reconstruction" << endl;
	}

	std::cout << "sdf fusion elapsed time: " << elapsed_time / getTickFrequency() << " sec" << std::endl;
	
	return model;
}

bool run_reconstruction(const Configuration &configuration, const vector<float> &volume)
{
	vector<RgbdFile> input_images = get_input_images(configuration.get_out_rgb_images_dir(), configuration.get_out_depth_images_dir());
	vector<Matrix4f> poses = read_scene_poses<float>(configuration.get_poses_file());

	if (input_images.size() != poses.size())
	{
		cerr << "Number of frames (" << input_images.size() << ") and number of poses (" << poses.size() << ") don't match" << endl;
		return false;
	}

	sdf model = reconstruct_sdf(configuration, volume, input_images, poses);
	vector<marching_cubes::triangle> model_mesh = mesh_valid_only(model);

	save_mesh_ply(model_mesh, configuration.get_model_file(), true);
	return true;
}
