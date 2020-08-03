//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include <opencv/cv.hpp>
#include <opencv2/viz.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>

using namespace std;
using namespace cv;

namespace fs = std::filesystem;

using json = nlohmann::json;

using ns = chrono::nanoseconds;
using sec = chrono::seconds;
using get_time = chrono::steady_clock;


#define CHECK_PARAM_EXISTS(parser, param_name) \
               if (!parser.has(param_name)) {\
                  cerr << "Missing parameter: " << param_name << endl;\
                  return EXIT_FAILURE;\
               }

#define CHECK_VALID_DIR(dir) \
      if (!fs::is_directory(dir)) {\
        cerr << "Invalid directory: " << dir << endl;\
        return EXIT_FAILURE;\
      }

struct ModelInfo
{
	int model_id;

	float diameter;

	float min_x;
	float min_y;
	float min_z;

	float size_x;
	float size_y;
	float size_z;
};

map<int, string> get_model_files(const string &models_dir)
{
	map<int, string> model_files;

	for (const auto& entry : fs::directory_iterator(models_dir))
	{
		const fs::path& file_path = entry.path();
		const string extension = file_path.extension().string();

		if (extension == ".ply")
		{
			string filename = file_path.filename().string();
			string raw_name = filename.substr(0, filename.find_last_of("."));
			string string_model_id = raw_name.substr(raw_name.find_last_of("obj_") + 1, raw_name.size());
			int model_id = std::stoi(string_model_id);
			model_files[model_id] = file_path.string();
		}
	}

	return model_files;
}

ModelInfo get_model_info_for(const viz::Mesh& mesh, int model_id)
{
	int num_vertices = mesh.cloud.cols;

	float min_x = 0.0f;
	float min_y = 0.0f;
	float min_z = 0.0f;

	float max_x = 0.0f;
	float max_y = 0.0f;
	float max_z = 0.0f;

	float diameter = 0.0f;

	for (int i = 0; i < num_vertices; ++i)
	{
		const Vec3f& v1 = mesh.cloud.at<Vec3f>(0, i);

		float x = v1(0);
		float y = v1(1);
		float z = v1(2);

		if (i == 0 || x < min_x)
		{
			min_x = x;
		}

		if (i == 0 || x > max_x)
		{
			max_x = x;
		}

		if (i == 0 || y < min_y)
		{
			min_y = y;
		}

		if (i == 0 || y > max_y)
		{
			max_y = y;
		}

		if (i == 0 || z < min_z)
		{
			min_z = z;
		}

		if (i == 0 || z > max_z)
		{
			max_z = z;
		}

		for (int j = 0; j < num_vertices; ++j)
		{
			const Vec3f& v2 = mesh.cloud.at<Vec3f>(0, j);
			float current_diameter = cv::norm(v1 - v2, NORM_L2);
			
			if (current_diameter > diameter)
			{
				diameter = current_diameter;
			}
		}
	}

	ModelInfo model_info = {};

	model_info.model_id = model_id;
	model_info.diameter = diameter;
	model_info.min_x = min_x;
	model_info.size_x = max_x - min_x;
	model_info.min_y = min_y;
	model_info.size_y = max_y - min_y;
	model_info.min_z = min_z;
	model_info.size_z = max_z - min_z;

	return model_info;
}

inline json model_info_to_json(const ModelInfo &model_info)
{
	json out;
	out["diameter"] = model_info.diameter;
	out["mix_x"] = model_info.min_x;
	out["min_y"] = model_info.min_y;
	out["min_z"] = model_info.min_z;
	out["size_x"] = model_info.size_x;
	out["size_y"] = model_info.size_y;
	out["size_z"] = model_info.size_z;
	return out;
}

void write_output(const string &models_info_file, const vector<ModelInfo> &model_infos)
{
	json out_json;

	for (const auto &model_info : model_infos)
	{
		out_json[to_string(model_info.model_id)] = model_info_to_json(model_info);
	}

	std::ofstream ofs(models_info_file);
	ofs << std::setw(4) << out_json << std::endl;
}


vector<ModelInfo> compute_model_infos(const map<int, string> &model_files)
{
	vector<ModelInfo> model_infos;

	for (auto const &[model_id, model_file]: model_files)
	{
		cout << "Computing model info for model #" << model_id << endl;
		auto start_time = get_time::now();

		const viz::Mesh mesh = viz::Mesh::load(model_file);
		model_infos.emplace_back(get_model_info_for(mesh, model_id));
		
		auto end_time = get_time::now();
		auto time_diff = end_time - start_time;
		cout << "Elapsed time:  " << chrono::duration_cast<sec>(time_diff).count() << " s " << endl;
	}

	std::sort(model_infos.begin(), model_infos.end(), [] (const auto &mi1, const auto &mi2)
	{
		return mi1.model_id < mi2.model_id;
	});

	return model_infos;
}

int main(int argc, char* argv[])
{
	const String arg_keys =
		"{help h |      | help message   }"
		"{models_dir           |     | models dir         }";

	cv::CommandLineParser parser(argc, argv, arg_keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	CHECK_PARAM_EXISTS(parser, "models_dir")
	string models_dir = parser.get<string>("models_dir");
	CHECK_VALID_DIR(models_dir)

	map<int, string> model_files;
	try
	{
		model_files = get_model_files(models_dir);
	}
	catch (const std::exception& e)
	{
		cerr << "Failed to read model files. " << e.what() << endl;
		return EXIT_FAILURE;
	}

	const vector<ModelInfo> model_infos = compute_model_infos(model_files);
	string out_file = (fs::path(models_dir) / "models_info.json").string();
	write_output(out_file, model_infos);

	return 0;
}
