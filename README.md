# Joint Perception and Planning For Efficient Obstacle Avoidance Using Stereo Vision

This repository contains a C++ implementation of JPP for local obstacle avoidance using stereo cameras. A ROS wrapper for JPP is also included in the `ROS/` 
folder. JPP uses disparity confidence checks on-demand instead of the traditional dense 3D reconstruction approach while performing obstacle avoidance during 
navigation planning, thus significantly saving computational cost.

- Author: [Sourish Ghosh](http://sourishghosh.com/)

## Publication

If you use this software in an academic work or find it relevant to your research, kindly cite:

```
@inproceedings{ghosh2017joint,
  doi = { 10.1109/IROS.2017.8202271 },
  url = { https://www.joydeepb.com/Publications/jpp.pdf },
  pages = { 1026--1031 },
  organization = { IEEE },
  year = { 2017 },
  booktitle = { Intelligent Robots and Systems (IROS), 2017 IEEE/RSJ International Conference on },
  author = { Sourish Ghosh and Joydeep Biswas },
  title = { Joint Perception And Planning For Efficient Obstacle Avoidance Using Stereo Vision },
}
```

Link to paper: [https://www.joydeepb.com/Publications/jpp.pdf](https://www.joydeepb.com/Publications/jpp.pdf)

## Dependencies

- A C++ compiler (*e.g.*, [GCC](http://gcc.gnu.org/))
- [cmake](http://www.cmake.org/cmake/resources/software.html)
- [popt](http://freecode.com/projects/popt)
- [libconfig](http://www.hyperrealm.com/libconfig/libconfig.html)
- [Boost](http://www.boost.org/)
- [OpenCV](https://github.com/opencv/opencv)
- [OpenMP](http://www.openmp.org/)

Use the following command to install dependencies:

```bash
$ sudo apt-get install g++ cmake libpopt-dev libconfig-dev libboost-all-dev libopencv-dev python-opencv gcc-multilib
```

For compiling and running the ROS wrapper, install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu).

## Compiling

### 1. Building JPP libraries and binaries

Clone the repository:

```bash
$ git clone https://github.com/umass-amrl/jpp
```

The script `build.sh` compiles the JPP library:

```bash
$ cd jpp
$ chmod +x build.sh
$ ./build.sh
```

### 2. Building JPP ROS

For compiling the ROS wrapper, `rosbuild` is used. Add the path of the ROS wrapper to `ROS_PACKAGE_PATH` and put the following line in your `.bashrc` file. 
Replace `PATH` by the actual path where you have cloned the repository:

```bash
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PATH/jpp/ROS
```

Execute the `build_ros.sh` script:

```bash
$ chmod +x build_ros.sh
$ ./build_ros.sh
```

## Running JPP on AMRL and KITTI Datasets

### 1. Download Datasets

The complete example data (AMRL and KITTI) along with calibration files can be found 
[here](https://greyhound.cs.umass.edu/owncloud/index.php/s/3g9AwCSkGi6LznK).

### 2. Running JPP

After compilation, the `jpp` binary file is store inside the `bin/` folder. For processing a single pair of stereo images, use:

```bash
$ ./bin/jpp -l [path/to/left/image] -r [path/to/right/image] -c [path/to/stereo/calibration/file] -j [path/to/jpp/config/file] -o [output_mode]
```

For processing multiple stereo pairs stored in a directory, use:

```bash
$ ./bin/jpp -n [number of pairs] -d [path/to/directory] -c [path/to/stereo/calibration/file] -j [path/to/jpp/config/file] -o [output_mode]
```

**Note:** stereo image pairs inside the directory must be named like this: `left1.jpg`, `left2.jpg`, ... , `right1.jpg`, `right2.jpg`, ...

For the example datasets, calibration files are stored in the `calibration/` folder and JPP configurations are stored in the `cfg/` folder. JPP operates on 
3 output modes (set by the `-o` flag) as of now: `astar`, `rrt`, and `debug` mode. Set the flag `-v 1` for generating visualizations.

```bash
Usage: jpp [OPTION...]
  -n, --num_imgs=NUM            Number of images to be processed
  -d, --img_dir=STR             Directory containing image pairs (set if n > 0)
  -l, --left_img=STR            Left image file name
  -r, --right_img=STR           Right image file name
  -c, --calib_file=STR          Stereo calibration file name
  -j, --jpp_config_file=STR     JPP config file name
  -o, --output=STR              Output - astar, rrt, debug
  -v, --visualize=NUM           Set v=1 for displaying visualizations
  -w, --write_files=NUM         Set w=1 for writing visualizations to files
```

For example, running JPP on the KITTI dataset in `astar` mode:

```bash
$ ./bin/jpp -n 33 -d KITTI/ -c calibration/kitti_2011_09_26.yml -j cfg/kitti.cfg -o astar -v 1
```

|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/astar7-vis.jpg)       | ![](dumps/astar7-path.jpg)|

Running JPP on the AMRL dataset in `rrt` mode:

```bash
$ ./bin/jpp -n 158 -d AMRL/ -c calibration/amrl_jackal_webcam_stereo.yml -j cfg/amrl.cfg -o rrt -v 1
```

|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/rrt73-vis.jpg)        | ![](dumps/rrt73-path.jpg) |

**Note:** Press any key to move on to the next image pair.

### 3. Running JPP ROS

Run the ROS node `navigation`:

```bash
$ rosrun jpp navigation -l [left/image/topic] -r [right/image/topic] -c [path/to/stereo/calibration/file] -j [path/to/jpp/config/file] -o [output_mode]
```

The same flags for displaying/writing visualizations can be used for the ROS node as well.

```bash
Usage: navigation [OPTION...]
  -l, --left_topic=STR              Left image topic name
  -r, --right_topic=STR             Right image topic name
  -c, --calib_file=STR              Stereo calibration file name
  -j, --jpp_config_file=STR         JPP config file name
  -o, --output=STR                  Output - astar, rrt, debug
  -v, --visualize=NUM               Set v=1 for displaying visualizations
  -w, --write_files=NUM             Set w=1 for writing visualizations to files
  -d, --dynamic_reconfigure=NUM     Set d=1 for enabling dynamic reconfigure
```

JPP configuration parameters can be changed realtime by using `rqt_reconfigure`:

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

Make sure you set the flag `-d 1` while using dynamic reconfigure.

## Running JPP on your Datasets

### 1. Stereo Calibration

To run JPP on your own data, you need to have a pair of calibrated stereo cameras. For stereo calibration it is recommended to use 
[this tool](https://github.com/sourishg/stereo-calibration). The `XR` and `XT` matrices in the calibration file are the transformation matrices from the left 
camera reference frame to the robot reference frame. These matrices depends on how the stereo camera is mounted on the robot. Initially after stereo 
calibration (using the tool mentioned) you will not have the `XR` and `XT` matrices in your calibration file. You need to manually calibrate them and add them 
to the calibration file. Also, you only need the following matrices in your calibration file: `K1`, `K2`, `D1`, `D2`, `R`, `T`, `XR`, and `XT`. An example 
calibration file can be found inside the `calibration/` folder.

If you cannot calibrate for `XR` and `XT` then just set them to the identity and zero matrices respectively. Then use this [stereo dense 
reconstruction](https://github.com/umass-amrl/stereo_dense_reconstruction) tool to visualize how the point cloud looks in the robot reference frame and 
visually align the ground plane with `z=0`.

### 2. Running JPP

JPP can be run in the same way as explained for the exmaple AMRL and KITTI datasets.

## License

This software is released under the [MIT license](LICENSE).
