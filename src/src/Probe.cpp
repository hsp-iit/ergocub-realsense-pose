#include <Probe.h>

#include <yarp/eigen/Eigen.h>

using namespace yarp::eigen;


Probe::Probe(const std::string& port_name)
{
    if (!port_.open(port_name))
        throw(std::runtime_error("Cannot open the port " + port_name + "."));
}


Probe::~Probe()
{
    port_.close();
}


void Probe::set_data(const Eigen::VectorXd& data)
{
    yarp::sig::Vector& data_yarp = port_.prepare();

    data_yarp.resize(data.size());
    toEigen(data_yarp) = data;

    port_.write();
}
