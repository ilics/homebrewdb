# The code for creation of a dataset for 6D pose estimation. The code was used to create the [HomebrewedDB](https://arxiv.org/abs/1904.03167 "Link to the paper") dataset.

## Requirements
The code uses C++17 and has been tested on Windows platform with MS Visual Studio 2017.
Here is the list of libraries with the versions the code has been tested with:

- OpenCV (with contrib modules) == 3.4.0 (BSD License: https://opencv.org/license/)
- VTK == 8.1.2 (BSD License: https://vtk.org/about/#license)
- Eigen == 3.3.4 (MPL2: http://eigen.tuxfamily.org/index.php?title=Main_Page#License)
- Ceres Solver == 1.14.0 (New BSD license: http://ceres-solver.org/license.html)
- Boost == 1.69 (Boost Software License - Version 1.0: https://www.boost.org/users/license.html)
- Qt == 5.13 ( GPL or LGPLv3 open source licenses)
- Glog == 0.3.3 (Apache License 2.0 -- https://github.com/golang/glog/blob/master/LICENSE)
- Gflags == 2.2.2 (BSD 3-Clause "New" or "Revised" License :https://github.com/gflags/gflags/blob/master/COPYING.txt)
- nlohmann/json == 3.4.0 (MIT License)
- yaml-cpp == 0.6.2 (MIT License)
- Halcon HDevelop == 18.11 Progress (Halcon Campus License: https://www.mvtec.com/company/on-campus/licenses/). 
Note that the parts of the SW using Hacon Library will not function unless you get Halcon Campus License from MVTec Software GmbH or 
their authorized dealers.)
                                       

For RGB-D images capturing the follwoing SDKs were used:
- OpenNI2, alternatively libfreenect (Apache License 2.0)
- Kinect For Windows SDK 2.0 (Microsoft Kinect for Windows Software Development Kit (SDK) 2.0, End User License Agreement:
                              https://download.microsoft.com/download/0/D/C/0DC5308E-36A7-4DCD-B299-B01CDFC8E345/Kinect-SDK2.0-EULA_en-US.pdf )
- Intel RealSense SDK 2.0 (Apache License, Version 2.0.)
- Kinect Azure SDK 1.3.0 (MIT License)
## Building

Having the required libraries installed and respective environment variables set, each sub-project can
be built using CMake and further compiled with Visual Studio.

Environment

#### rgbd_capture
This project is composed of two modules: `librgbdcapture` and `app`. First, build `librgbdcapture`. Then, to build 
`app`, place the respective binaries of `librgbdcapture` into `app/librgbdcapture/lib` and `app/librgbdcapture/bin`.
#### refiner
This project depends on `libqtoffscreenrenderer`. Build and compile `libqtoffscreenrenderer` and place its respective
binaries into `refiner/third-party/librenderer/lib` `refiner/third-party/librenderer/bin`, then build and compile `refiner`.
#### gtwriter
This project depends on `libqtoffscreenrenderer`. Build and compile `libqtoffscreenrenderer` and place its respective
binaries into `gtwriter/third-party/librenderer/lib` `gtwriter/third-party/librenderer/bin`, then build and compile `gtwriter`.

Other subprojects do not require any particular handling to be built.

## Usage

#### rgbd_capture/app
Used to capture sequences of rgb-d images. Captured images are stored into respective directories `<out_dir>/rgb` and `<out_dir>/depth`
```
{help h       |          | help message   }		
{camera       |          | RGB-D camera (kinect, kinect2, realsense or kinect_azure)}
{num_frames   |          | number of frames to capture}
{async_save   |          | save frames asynchronously or store in memory while capturing}
{out_dir      |          | output directory for images}
{timeout      |          | timeout between frames in ms (for non-async capturing)}
```

### sdf_fusion
Used to filter frames with correctly estimated camera poses. Stores correct poses and respective images, and then performs 
a dense 3D reconstruction of the scene. Outputs `camera_poses.txt`, `images` directory with valid rgb and depth images and 
a dense reconstruction of the scene in `model.ply` file.
```
{help h usage ?  |          | help on usage}
{images_dir      |          | input images dir containing /rgb and /depth dirs}
{config_dir      |          | config dir containing board.yml and dict.yml for AruCo markerboard}
{intrinsics_file |          | file containing row-wise intrinsic matrix}
{marker_size     |  0.0491  | marker size of the markerboard (in meters)}
{voxel_size      |  0.002   | voxel size}
{preprocess      |          | do pose estimation and discarding of frames }
{req_num_frames  |          | out number of frames to perform reconstruction}
{out_dir         |          | out directory for storing camera poses, filtered images, reconstructed mesh}
```

### ppfshapematchinghalcon
Used to estimate poses of 3D models in the densely reconstructed scene. Reference models used for refinement must be stored in a format `obj_xxxxx.ply` in meters. Poses are ouputed 
into scene out directory `<scene_dir>/object_poses` in format `xxxxxx.txt` and stored as a 4x4 matrix. 
`scene_dir` must be a directory where the output of **sdf_fusion** is stored.
```
{ help h usage ?  |          | help on usage }
{ref_models_dir   |          | reference models directory}
{scene_dir        |          | directory containing the scene}
{model_ids        |          | comma-separated model ids}
{rel_smp_dist     |   0.05   | relative sampling distance}
{kp_frac          |   0.4    | keypoint fraction}
```

### refiner
Use to refine poses using multi-view edge based refinement on rgb images. 
`scene_dir` must be the same directory as used by **ppfshapematchinghalcon**, where the output of 
**sdf_fusion** is stored.
```
{help h usage ? |          | help on usage }
{scene_dir      |          | path to the scene where poses must be refined}
{model_ids      |     -1   | model ids to be refined, default -1, i.e. refinement for all object_poses}
{config_file    |          | path to the json config file (e.g. see refiner/config/config.json}
```

### gtwriter/scene-gt-writer
Used to write pose ground truth labels for scenes.
```
{help h               |         | help message}
{intrinsics_file      |         | intrinsics file}
{reference_models_dir |         | directory with reference models}
{scenes_dirs          |         | comma separated scene directories, which will be joined}
{out_dir              |         | output directory}
{copy_images          |         | should re-index images and copy the do the output dir}
```

### gtwriter/model-info-writer
Used to compute meta-info for the 3D models.
```
{help h      |         | help message }
{models_dir  |         | models dir}
```

### refine-depth
A utility used to estimate a first degree polynomial correction function for depth. The correction function
treats depth in meters. The images must capture an empty AruCo markerboard at different distances and elevations.

```
{help h          |         | help message}
{images_dir      |         | input images dir}
{config_dir      |         | configuration dir with board.yml and dict.yml files for AruCo markerboard}
{intrinsics_file |         | intrinsics file}
{marker_size     |         | marker size in meters}
```


