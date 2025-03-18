# ergocub-realsense-pose

### Requirements

- [YARP](https://yarp.it/)
- [iDynTree](https://github.com/robotology/idyntree)
- [Eigen](https://eigen.tuxfamily.org)
- [ergocub-software](https://github.com/icub-tech-iit/ergocub-software)

### How to setup on ergoCub robot
1) ssh to the torso PC `ssh -X ergocub-torso`
2) go under hsp folder: `cd $ROBOT_CODE/hsp` where $ROBOT_CODE is /usr/local/src/robot
3) type the following commands:

```console
git clone https://github.com/hsp-iit/ergocub-realsense-pose
cd ergocub-realsense-pose
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/src/robot/robotology-superbuild/build/install ..
make install
```

### How to run
```console
ergocub-rs-pose $YARP_ROBOT_NAME /usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/$YARP_ROBOT_NAME/model.urdf
```

The executable will stream a `x y z axis_x axis_y axis_z angle` vector to the port `/ergocub-rs-pose/pose:o`.
