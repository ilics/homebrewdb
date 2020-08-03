//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "sequence_composer.h"
#include <iostream>
#include "scene.h"
#include <io_utils.hpp>

using namespace std;
using namespace Eigen;

#define CHECK_PARAM_EXISTS(parser, param_name) \
      if (!parser.has(param_name)) {\
        cerr << "Missing parameter: " << param_name << endl;\
        return false;\
      }

#define CHECK_VALID_DIR(dir) \
      if (!fs::is_directory(dir)) {\
        cerr << "Invalid directory: " << dir << endl;\
        return false;\
      }

#define CHECK_VALID_FILE(f) \
     if (!fs::is_regular_file(f)) {\
       cerr << "Invalid file: " << f << endl;\
       return false;\
     }


string get_model_file(const string& reference_models_dir, int model_id)
{
	std::stringstream filename_ss;
	filename_ss << "obj_" << std::setfill('0') << std::setw(6) << model_id << ".ply";
	const std::string filename = filename_ss.str();

	return (fs::path(reference_models_dir) / filename).string();
}

bool get_scene_models(const string& refined_object_poses_dir, const string& reference_models_dir,
                      vector<Model>& scene_models)
{
	for (const auto& entry : fs::directory_iterator(refined_object_poses_dir))
	{
		const fs::path& file_path = entry.path();
		const string extension = file_path.extension().string();

		if (extension == ".txt")
		{
			string filename = file_path.filename().string();
			string raw_name = filename.substr(0, filename.find_last_of("."));

			int model_id = std::stoi(raw_name);
			string model_file = get_model_file(reference_models_dir, model_id);
			CHECK_VALID_FILE(model_file)

			Matrix4f model_pose;
			try
			{
				model_pose = read_pose_from_file<float>(file_path.string());
			}
			catch (const std::exception&)
			{
				cerr << "Failed to read pose for model #" << model_id << ", scene dir: " << entry << endl;
				return false;
			}

			scene_models.emplace_back(Model(model_file, model_pose, model_id));
		}
	}

	if (scene_models.empty())
	{
		cerr << "No models poses for scene: " << refined_object_poses_dir << endl;
		return false;
	}

	std::sort(scene_models.begin(), scene_models.end(), [](const Model& m1, const Model& m2)
	{
		return m1.model_id < m2.model_id;
	});

	return true;
}


bool verify_joined_scene_models(const vector<vector<Model>>& joined_scene_models)
{
	if (joined_scene_models.empty())
	{
		cerr << "No scenes to join." << endl;
		return false;
	}

	if (joined_scene_models.size() == 1)
	{
		// no consistency expected
		return true;
	}

	const vector<Model>& first_scene_models = joined_scene_models[0];

	for (size_t i = 1; i < joined_scene_models.size(); ++i)
	{
		const vector<Model>& current_scene_models = joined_scene_models[i];

		if (first_scene_models.size() != current_scene_models.size())
		{
			cerr << "Invalid number of poses for scenes to be joined" << endl;
			return false;
		}

		for (size_t j = 0; j < first_scene_models.size(); ++j)
		{
			const Model& m1 = first_scene_models[j];
			const Model& m2 = current_scene_models[j];

			if (m1.model_id != m2.model_id)
			{
				cerr << "Mismatching model ids: " << m1.model_id << " and " << m2.model_id << endl;
				return false;
			}
		}
	}

	return true;
}

bool get_joined_scene_models(const vector<string>& scene_dirs, const string& reference_models_dir,
                             vector<vector<Model>>& joined_scene_models)
{
	for (const auto& scene_dir : scene_dirs)
	{
		vector<Model> scene_models;

		string refined_object_poses_dir = (fs::path(scene_dir) / "refined_object_poses").string();
		CHECK_VALID_DIR(refined_object_poses_dir)

		if (!get_scene_models(refined_object_poses_dir, reference_models_dir, scene_models)) return false;
		joined_scene_models.push_back(scene_models);
	}

	return verify_joined_scene_models(joined_scene_models);
}

bool compose_scenes(const Configuration& configuration, vector<Scene>& scenes)
{
	const vector<string>& scenes_dirs = configuration.get_scenes_dirs();
	vector<vector<Model>> joined_scene_models;

	if (!get_joined_scene_models(scenes_dirs, configuration.get_reference_models_dir(), joined_scene_models)) return
		false;

	for (size_t i = 0; i < scenes_dirs.size(); ++i)
	{
		const auto& scene_dir = scenes_dirs[i];
		string camera_poses_file = (fs::path(scene_dir) / "camera_poses.txt").string();
		CHECK_VALID_FILE(camera_poses_file)

		vector<Matrix4f> camera_poses;
		try
		{
			camera_poses = read_scene_poses<float>(camera_poses_file);
		}
		catch (const std::exception&)
		{
			cerr << "Failed to read camera poses: " << camera_poses_file << endl;
			return false;
		}
		scenes.emplace_back(Scene(configuration, scene_dir, joined_scene_models[i], camera_poses));
	}

	return true;
}


bool compose_joined_sequence(const Configuration& configuration, std::vector<Frame>& joined_sequences)
{
	vector<Scene> scenes;
	if (!compose_scenes(configuration, scenes)) return false;

	size_t total_number_of_frames = 0;

	for (auto& scene : scenes) total_number_of_frames += scene.get_number_of_frames();
	joined_sequences.reserve(total_number_of_frames);

	for (auto& scene : scenes)
	{
		auto scene_frames = scene.convert_to_scene_frames();
		joined_sequences.insert(joined_sequences.end(), scene_frames.begin(), scene_frames.end());
	}

	return true;
}
