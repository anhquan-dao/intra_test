#include <intra_test/shm_tp_lib_cpp.h>
#include <boost/python.hpp>

BOOST_PYTHON_MODULE(SHMTransportCPP)
{
    using namespace boost::python;
    using namespace intraprocess_test;

    class_<SHMSubscriberCPP<nav_msgs::OccupancyGrid>>
    ("SubscriberOccupancyGridSHM", init<std::string>())
    ;

    class_<SHMPublisherCPP<nav_msgs::OccupancyGrid>>
    ("PublisherOccupancyGridSHM", init<std::string>())
    .def("publish", &SHMPublisherCPP<nav_msgs::OccupancyGrid>::pypublish)
    ;
}