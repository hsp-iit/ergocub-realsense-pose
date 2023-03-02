#ifndef PROBE_H_
#define PROBE_H_

#include <Eigen/Dense>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>


class Probe
{
public:
    Probe(const std::string& port_name);

    ~Probe();

    void set_data(const Eigen::VectorXd& data);

private:
    yarp::os::BufferedPort<yarp::sig::Vector> port_;

    const std::string log_prefix_ = "Probe";
};

#endif /* PROBE_H_ */
