/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <filesystem>

#include <Eigen/Dense>

#include <ergoCubHeadEncoders.h>
#include <ForwardKinematicsiDynTree.h>
#include <Probe.h>

#include <yarp/os/Network.h>

using namespace Eigen;
using namespace std::literals::chrono_literals;


int main(int argc, char** argv)
{
    const std::string log_prefix = "ergocub-rs-pose";
    std::string robot_name = "ergocub";
    std::string urdf_path;

    if (argc != 3)
    {
        // Automatically get the path to urdf based on robot name env var
		const char* env_var = std::getenv("YARP_ROBOT_NAME");
    	std::string robot_name = (env_var != nullptr) ? env_var : ""; 
		if (!robot_name.empty()) 
		{
        	urdf_path = "/usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/" + robot_name + "/model.urdf";
			std::cerr << "[ergocub-rs-pose] " << "Robot Port prefix name and path to URDF are required. USING DEFAULT ONES:\n" <<
				  "robot_name " << robot_name << "\n" <<
				  "urdf_path " << urdf_path << std::endl;
            std::cerr << "Synopsis: " + log_prefix + " <robot_name> <urdf_path>" << std::endl << std::endl;
            // check if file exist:
            if (!std::filesystem::exists(urdf_path))
            {
                std::cerr << "[ergocub-rs-pose] the urdf file path does not exist: " << urdf_path << std::endl;
                return 1;
            }
    	}
		else
		{
			std::cerr << "[ergocub-rs-pose] " << " YARP_ROBOT_NAME not set, aborting...";
			return 1;
		}
    }
    else
    {
        robot_name = argv[1];
        urdf_path = argv[2];
    }
    
    const std::string root_frame_name = "root_link";
    const std::string ee_frame_name = "realsense_rgb_frame";

    yarp::os::Network network;
    if (!network.checkNetwork())
        throw(std::runtime_error(log_prefix + "::main(). Error: cannot find YARP network."));

    /* Set list of joints to be used. */
    std::vector<std::string> list_joints =
    {
        "torso_roll",
        "torso_pitch",
        "torso_yaw",
        "neck_pitch",
        "neck_roll",
        "neck_yaw",
        "camera_tilt"
    };

    /* Instantiate iDynTree-based forward kinematics. */
    ForwardKinematicsiDynTree fk(urdf_path, list_joints, root_frame_name, ee_frame_name);

    /* Instantiate encoders. */
    ergoCubHeadEncoders encoders(robot_name, log_prefix);

    /* Instantiate output probe. */
    Probe probe("/" + log_prefix + "/pose:o");

    /* Update loop. */
    while (true)
    {
        /* Read encoders. */
        auto [valid_encoders, joints_angles] = encoders.joints();

        if (valid_encoders)
        {
            /* Set joints and update fk. */
            fk.set_joints_state(joints_angles);
            fk.update();

            /* Get forward kinematics. */
            Transform<double, 3, Affine> transform = fk.get_ee_transform();

            /* Convert as a vector of length 7 containing x, y, z, axis-x, axis-y, axis-z, angle. */
            AngleAxisd axis_angle(transform.rotation());

            VectorXd configuration_vector;
            configuration_vector.resize(7);
            configuration_vector.head<3>() = transform.translation();
            configuration_vector.segment<3>(3) = axis_angle.axis();
            configuration_vector(6) = axis_angle.angle();

            /* Send to output via YARP port. */
            probe.set_data(configuration_vector);
        }

        std::this_thread::sleep_for(33ms);
    }

    return EXIT_SUCCESS;
}
