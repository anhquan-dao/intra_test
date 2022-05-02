#include <intra_test/shared_resource.h>
void hello()
{
    std::cout << "Hello Python. I'm C++, looking forward to work with you!" << std::endl;
}


// name has to match with the name in CMakeLists `add_library` + prefix 'lib'
BOOST_PYTHON_MODULE(mycpplib2)
{
    using namespace boost::python;
    using namespace intraprocess_test;
    def("hello", hello);
    class_<SharedResource, boost::shared_ptr<SharedResource>, boost::noncopyable>
    ("SharedResource", no_init)
        .def("make", &SharedResource::make)
        .staticmethod("make")
        .def("get_count", &SharedResource::get_count)
        .staticmethod("get_count")
    ;
}


