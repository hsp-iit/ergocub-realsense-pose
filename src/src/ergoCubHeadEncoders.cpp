#include <ergoCubHeadEncoders.h>

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include <iostream>

using namespace Eigen;
using namespace yarp::os;


ergoCubHeadEncoders::ergoCubHeadEncoders(const std::string& robot_name, const std::string& port_prefix)
{
    /* Check YARP network. */
    if (!network_.checkNetwork())
        throw(std::runtime_error(log_prefix_ + "::ctor. Error: YARP is not available."));

    {
        Property properties;
        properties.put("device", "remote_controlboard");
        properties.put("local", "/" + port_prefix + "/torso/");
        properties.put("remote", "/" + robot_name + "/torso");

        /* Try to open the driver. */
        if (!driver_torso_.open(properties))
            throw(std::runtime_error(log_prefix_ + "::ctor. Error: cannot open the torso driver."));

        if (!(driver_torso_.view(encoders_torso_) && (encoders_torso_ != nullptr)))
        {
            throw(std::runtime_error(log_prefix_ + "::ctor. Error: cannot open the torso position view."));
        }
    }

    {
        Property properties;
        properties.put("device", "remote_controlboard");
        properties.put("local", "/" + port_prefix + "/head/");
        properties.put("remote", "/" + robot_name + "/head");

        /* Try to open the driver. */
        if (!driver_head_.open(properties))
            throw(std::runtime_error(log_prefix_ + "::ctor. Error: cannot open the head driver."));

        if (!(driver_head_.view(encoders_head_) && (encoders_head_ != nullptr)))
        {
            throw(std::runtime_error(log_prefix_ + "::ctor. Error: cannot open the head position view."));
        }
    }
}


std::tuple<bool, VectorXd> ergoCubHeadEncoders::joints() const
{
    VectorXd joints;

    /* Get torso encoders. */
    int number_joints_torso;
    encoders_torso_->getAxes(&number_joints_torso);

    yarp::sig::Vector joints_torso;
    joints_torso.resize(number_joints_torso);

    if (!encoders_torso_->getEncoders(joints_torso.data()))
        return std::make_tuple(false, joints);

    /* Get head encoders. */
    int number_joints_head;
    encoders_head_->getAxes(&number_joints_head);

    yarp::sig::Vector joints_head;
    joints_head.resize(number_joints_head);

    if (!encoders_head_->getEncoders(joints_head.data()))
        return std::make_tuple(false, joints);

    /* Assemble output vector. */
    joints.resize(number_joints_torso + number_joints_head);
    for (std::size_t i = 0; i < number_joints_torso; i++)
        joints(i) = joints_torso(i);
    for (std::size_t i = 0; i < number_joints_head; i++)
        joints(i + 3) = joints_head(i);

    joints *= M_PI / 180.0;

    return std::make_tuple(true, joints);
}


void ergoCubHeadEncoders::close()
{
    if (driver_torso_.isValid())
        driver_torso_.close();

    if (driver_head_.isValid())
        driver_head_.close();
}
