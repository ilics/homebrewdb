//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <filesystem>
#include <random>
#include <algorithm>
#include <unordered_set>

#include "input_preprocessor.hpp"
#include "pose_validator.hpp"
#include "io_utils.hpp"
#include "marker_detector.hpp"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;


MarkerDetector setup_marker_detector(const Configuration &configuration)
{
	MarkerDetector markers;
	markers.init(configuration.get_intrinsics());
	markers.loadDictionary(configuration.get_dictionary_file());
	markers.loadBoard(configuration.get_board_file());
	markers.setThresholdParams(9, 5); // this is the function that controls the edge detection parameters

	return markers;
}

unordered_set<int> get_indices_to_discard(int number_valid_frames, int min_number_required_frames)
{
	//sample random frames to remove
	vector<int> indices(static_cast<size_t>(number_valid_frames));
	size_t n(0);
	generate(begin(indices), end(indices), [&] { return n++; });

	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(indices.begin(), indices.end(), g);

	const int num_of_frames_to_discard = number_valid_frames - min_number_required_frames;
	unordered_set<int> indices_to_discard(indices.begin(), indices.begin() + num_of_frames_to_discard);
	return indices_to_discard;
}

void store_sampled_images(const vector<RgbdFile> &rgbd_files, const vector<int> &selected_indices, const Configuration &configuration)
{
	vector<RgbdFile> selected_rgbd_files(selected_indices.size());
	transform(selected_indices.begin(), selected_indices.end(), selected_rgbd_files.begin(), [&rgbd_files](int i)
	{
		return rgbd_files[i];
	});

	store_images(selected_rgbd_files, configuration.get_out_rgb_images_dir(), configuration.get_out_depth_images_dir());
}

bool store_results(const Configuration& configuration, const vector<RgbdFile> &rgbd_files,
	const vector<Isometry3f> &valid_poses, const vector<int> &valid_frame_indices)
{
	const int required_number_of_frames = configuration.get_required_number_of_frames();
	if (valid_frame_indices.size() < static_cast<size_t>(required_number_of_frames))
	{
		cerr << "Insufficient number of frames for sequence";
		return false;	
	}

	const int number_valid_frames = valid_frame_indices.size();
	unordered_set<int> indices_to_discard = get_indices_to_discard(number_valid_frames, required_number_of_frames);

	vector<Isometry3f> subsampled_valid_poses;
	vector<int> subsampled_valid_frame_indices;

	for (int i = 0; i < number_valid_frames; ++i) {
		if (indices_to_discard.find(i) == indices_to_discard.end()) {
			subsampled_valid_frame_indices.push_back(valid_frame_indices[i]);
			subsampled_valid_poses.push_back(valid_poses[i]);
		}
	}

	store_poses(subsampled_valid_poses, configuration.get_poses_file());
	store_sampled_images(rgbd_files, subsampled_valid_frame_indices, configuration);

	return true;
}

bool preprocess_input(const Configuration& configuration, const vector<float> &volume)
{
	const float marker_size = configuration.get_marker_size();

	MarkerDetector markers = setup_marker_detector(configuration);
	vector<RgbdFile> rgbd_files = get_input_images(configuration.get_in_rgb_images_dir(), configuration.get_in_depth_images_dir());

	const int required_number_of_frames = configuration.get_required_number_of_frames();

	if (rgbd_files.size() < static_cast<size_t>(required_number_of_frames))
	{
		cerr << "Insufficient number of frames for sequence" << endl;
		return false;
	}

	int current_frame_index = 0;

	vector<Isometry3f> valid_poses;
	vector<int> valid_frame_indices;

	for (const auto &rgbd_file : rgbd_files)
	{
		RgbdFrame rgbd_frame = get_rgbd_frame(rgbd_file);
		
		const Mat &rgb_image = rgbd_frame.first;
		const Mat &depth_image = rgbd_frame.second;

		cout << "Estimating camera pose for image " << rgbd_file.first << endl;
		
		if (markers.detect(rgb_image, marker_size) == 0) {
			std::cerr << "Could not estimate the markerboard pose from image #" << fs::path(rgbd_file.first).filename() << std::endl;
			current_frame_index++;
			continue;
		}

		if (markers.getNumDetected() < 6)
		{
			std::cerr << "Insufficient number of markers detected for image #" << fs::path(rgbd_file.first).filename() << std::endl;
			current_frame_index++;
			continue;
		}

		Isometry3f pose = markers.board.pose;
		PoseValidationPayload payload;

		payload.intrinsics = configuration.get_intrinsics();
		payload.pose = pose;
		payload.rgb_image = rgb_image;
		payload.depth_image = depth_image;
		payload.volume = volume;

		const bool is_valid_pose = validate_pose(payload);

		if (is_valid_pose)
		{
			valid_frame_indices.push_back(current_frame_index);
			valid_poses.push_back(pose);
		} else
		{
			cout << "Invalid poses for image #" << fs::path(rgbd_file.first).filename() << ", skipping" << endl;
		}

		current_frame_index++;
	}

	return store_results(configuration, rgbd_files, valid_poses, valid_frame_indices);
}
