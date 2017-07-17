# StereoV2PointCloud_pipeline
Scripts and a general pipeline for
* Rectifies and calibrates inputs from stereo cameras
* Generate disparity maps from rectified and calibrated stereo inputs
* Generate point clouds from disparity maps for use in autonomous navigation system

# Todo
* Optimize and strip down the third-party scripts to increase rectification, calibration and disparity-map-generation speed to something more reasonable
* Time point cloud generation to see whether live-time point cloud generation is practical
* Isolate dependencies and create automatic installation/verification scripts
* Implement unit tests

## Instructions

While in subdirectory, run `cmake .. && make && ./stereoToGrid`. Or, you can run the `reinit.sh` script under the `/testing` example directory.

## Installation

Required libraries:
* OpenCV
* BOOST
* libConfig
* PCL (optional in this version)


# License

Copyright Eric Zhao.

Third-party code is found under /src/dispCloud/thirdParty.
