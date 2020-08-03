//######################################################################
//#   Librgbdcapture Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include <iomanip>

#include <vector>
#include <string>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include "rgbd_camera.h"
#include <thread>
#include <chrono>
#include <deque>
#include <future>
#include <mutex>
#include <condition_variable>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

#define CHECK_PARAM_EXISTS(parser, param_name) \
                if (!parser.has(param_name)) {\
                   cerr << "Missing parameter: " << param_name << endl;\
                   return false;\
                }

#define CHECK_VALID_FILE(f) \
     if (!fs::is_regular_file(f)) {\
       cerr << "Invalid file: " << f << endl;\
       return false;\
     }

#define CHECK_VALID_DIR(dir) \
      if (!fs::is_directory(dir)) {\
        cerr << "Invalid directory: " << dir << endl;\
        return false;\
      }

struct Configuration
{
	string camera_name;
	int num_frames;
	bool async_save;
	string rgb_out_dir;
	string depth_out_dir;
	int timeout_ms;
};

struct Frame
{
	cv::Mat bgr;
	cv::Mat depth;
	int index;
};

void save_images(const Configuration& configuration, const Frame& frame)
{
	stringstream filename_ss;

	filename_ss << setfill('0') << setw(6) << frame.index << ".png";
	string filename = filename_ss.str();

	string rgb_file = (fs::path(configuration.rgb_out_dir) / filename).string();
	string depth_file = (fs::path(configuration.depth_out_dir) / filename).string();

	cv::Mat depth_mm;

	frame.depth.convertTo(depth_mm, CV_32FC1, 1000);
	depth_mm.convertTo(depth_mm, CV_16UC1);

	imwrite(rgb_file, frame.bgr);
	imwrite(depth_file, depth_mm);
}

inline void create_if_not_exists(const string& dir)
{
	if (!fs::is_directory(dir)) fs::create_directories(dir);
}

const string arg_keys =
	"{help h |     | help message   }"
	"{camera |     | RGB-D camera (kinect, kinect2, realsense or kinect_azure)}"
	"{num_frames           |     | number of frames to capture         }"
	"{async_save           |     | save frames asynchronously or store in memory while capturing}"
	"{out_dir           |     | Output directory for images}"
	"{timeout           |     | timeout between frames in ms (for non-async capturing)}";

bool parse_configuration(int argc, char** argv, Configuration& configuration)
{
	cv::CommandLineParser parser(argc, argv, arg_keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		return false;
	}

	CHECK_PARAM_EXISTS(parser, "camera")
	string camera_name = parser.get<string>("camera");

	CHECK_PARAM_EXISTS(parser, "num_frames")
	int num_frames = parser.get<int>("num_frames");

	if (num_frames <= 0)
	{
		std::cerr << "Invalid number of frames: " << num_frames << std::endl;
		return false;
	}

	CHECK_PARAM_EXISTS(parser, "out_dir")
	string out_dir = parser.get<string>("out_dir");

	string rgb_out_dir = (fs::path(out_dir) / "rgb").string();
	string depth_out_dir = (fs::path(out_dir) / "depth").string();

	create_if_not_exists(rgb_out_dir);
	create_if_not_exists(depth_out_dir);

	bool asynch_save = parser.has("async_save");

	int timeout_ms;
	if (parser.has("timeout"))
	{
		timeout_ms = parser.get<int>("timeout");

		if (timeout_ms <= 0)
		{
			std::cerr << "Invalid timeout!" << std::endl;
			return false;
		}
	}
	else
	{
		timeout_ms = asynch_save ? 5 : 150;
	}

	configuration.rgb_out_dir = rgb_out_dir;
	configuration.depth_out_dir = depth_out_dir;
	configuration.async_save = asynch_save;
	configuration.num_frames = num_frames;
	configuration.camera_name = camera_name;
	configuration.timeout_ms = timeout_ms;
	return true;
}

void capture_with_asynch_save(const Configuration& configuration)
{
	mutex mutex;
	condition_variable condition;
	bool more_to_process = true;
	deque<Frame> to_process;

	auto process = async(launch::async, [&]()
	{
		unique_ptr<RGBDCameraInterface> camera(get_rgbd_camera(configuration.camera_name));

		if (!camera)
		{
			std::cout << "Unknown camera: " << configuration.camera_name << std::endl;
			exit(EXIT_FAILURE);
		}

		typedef lock_guard<std::mutex> lock_type;

		int img_id = 0;

		auto start_time = cv::getTickCount();

		while (img_id < configuration.num_frames)
		{
			if (!camera->get_frames_live()) continue;

			Frame f = {camera->get_bgr().clone(), camera->get_depth_float().clone(), img_id};

			imshow("Depth image", f.depth);
			imshow("Color image", f.bgr);
			cv::waitKey(configuration.timeout_ms);

			lock_type lock(mutex);
			to_process.push_back(f);

			condition.notify_one();

			img_id++;
		}

		int64 elapsed_time = cv::getTickCount() - start_time;
		std::cout << "Time taken for capturing: " << elapsed_time / cv::getTickFrequency() << " sec" << std::endl;

		lock_type lock(mutex);
		more_to_process = false;
		condition.notify_one();
	});

	while (true)
	{
		unique_lock<std::mutex> lock(mutex);
		condition.wait(lock, [&]
		{
			return !more_to_process || !to_process.empty();
		});

		if (!more_to_process && to_process.empty())
		{
			break;
		}
		if (to_process.empty())
		{
			continue;
		}

		Frame f = move(to_process.front());

		save_images(configuration, f);
		to_process.pop_front();
	}

	process.get();
}


int main(int argc, char** argv)
{
	Configuration configuration;
	if (!parse_configuration(argc, argv, configuration)) return EXIT_FAILURE;

	bool do_capture = true;

	std::cout << "Capturing setup: camera=" << configuration.camera_name << ", asynch save=" <<
		configuration.async_save << ", timeout (ms)=" << configuration.timeout_ms << ", number of frames=" <<
		configuration.num_frames << ", out rgb directory=" << configuration.rgb_out_dir << ", out depth directory=" <<
		configuration.depth_out_dir << std::endl;


	if (configuration.async_save)
	{
		capture_with_asynch_save(configuration);
	}
	else
	{
		unique_ptr<RGBDCameraInterface> camera(get_rgbd_camera(configuration.camera_name));

		if (!camera)
		{
			std::cout << "Unknown camera: " << configuration.camera_name << std::endl;
			return EXIT_FAILURE;
		}

		vector<Frame> captured_frames;

		int img_id = 0;

		auto start_time = cv::getTickCount();

		while (do_capture && img_id < configuration.num_frames)
		{
			if (!camera->get_frames_live()) continue;

			Frame f = {camera->get_bgr().clone(), camera->get_depth_float().clone(), img_id};

			imshow("Depth image", f.depth);
			imshow("Color image", f.bgr);

			captured_frames.push_back(f);

			std::cout << "Captured image #" << img_id << std::endl;

			int key = cv::waitKey(configuration.timeout_ms);

			switch (key)
			{
			case 'q':
				do_capture = false;
				break;
			case 'c':
				break;
			default:
				break;
			}
			img_id++;
		}

		int64 elapsed_time = cv::getTickCount() - start_time;
		std::cout << "Time taken for capturing: " << elapsed_time / cv::getTickFrequency() << " sec" << std::endl;

		std::cout << "Saving images..." << std::endl;

		for (auto& f : captured_frames)
		{
			save_images(configuration, f);
		}
	}


	return 0;
}
