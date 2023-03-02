#ifndef ERGOCUB_HEAD_ENCODERS_H
#define ERGOCUB_HEAD_ENCODERS_H

#include <Eigen/Dense>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

class ergoCubHeadEncoders
{
public:
    ergoCubHeadEncoders(const std::string& robot_name, const std::string& port_prefix);

    std::tuple<bool, Eigen::VectorXd> joints() const;

    void close();

private:
    yarp::dev::PolyDriver driver_torso_;

    yarp::dev::PolyDriver driver_head_;

    yarp::dev::IEncoders* encoders_torso_;

    yarp::dev::IEncoders* encoders_head_;

    yarp::os::Network network_;

    const std::string log_prefix_ = "ergoCubHeadEncoders";
};

#endif /* ECUB_HEAD_H */
