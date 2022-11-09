# dtsdf_ros

This package is a ROS wrapper for the Directional TSDF implementation https://github.com/AIS-Bonn/DirectionalTSDF

## Build Notes
The DirectionalTSDF code is included as git submodule, so please ensure all submodules are initialized.
```bash
cd path/to/catkin_ws/src/
git clone --recurse-submodules https://github.com/AIS-Bonn/dtsdf_ros
catkin build
```

Also, make sure all [requirements](https://github.com/AIS-Bonn/DirectionalTSDF#131-requirements) are installed.
Alternatively, you can set up a conda environment, as described [here](https://github.com/AIS-Bonn/DirectionalTSDF#11-anaconda).