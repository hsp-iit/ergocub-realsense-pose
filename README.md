# ergocub-rs-pose-streamer

### Requirements

- [YARP](https://yarp.it/)
- [iDynTree](https://github.com/robotology/idyntree)
- [Eigen](https://eigen.tuxfamily.org)
- [ergocub-software](https://github.com/icub-tech-iit/ergocub-software)

### How to build

```console
git clone https://github.com/ergoCub-HSP/ergocub-rs-pose-streamer
cd ergocub-rs-pose-streamer
mkdir build
cd build
cmake [-DCMAKE_INSTALL_PREFIX=<valid_install_prefix>] ../
make [install]
```

### How to run
```console
ergocub-rs-pose <robot_name> <path_to_ergocub-software>/urdf/ergoCub/robots/<model_name>/model.urdf
```

The executable will stream a `x y z axis_x axis_y axis_z angle` vector to the port `/ergocub-rs-pose/pose:o`.
