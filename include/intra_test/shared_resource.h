#ifndef SHARED_RESOURCE
#define SHARED_RESOURCE

#include <iostream>
#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <opencv2/opencv.hpp>

namespace intraprocess_test{
    class SharedResource : public boost::enable_shared_from_this<SharedResource>{
        public:
            SharedResource() = delete;
            ~SharedResource()
            { 
                printf("SharedResource obj %p deconstructed", this);
            }

            static boost::shared_ptr<SharedResource> make()
            {
                if (test_weak_ptr.use_count()) {
                return test_weak_ptr.lock(); 
                } else {
                auto strong = boost::shared_ptr<SharedResource>(new SharedResource(ctor_passkey()));
                test_weak_ptr = strong;
                return strong;
                }
            }

            static size_t get_count()
            {
                printf("  Resource.g_weak.use_count() = %ld\n", test_weak_ptr.use_count());
                return test_weak_ptr.use_count();
            }
        private:
            boost::shared_ptr<cv::Mat> test_ptr{new cv::Mat(100,100,CV_32FC1)};
            static boost::weak_ptr<SharedResource> test_weak_ptr;

            struct ctor_passkey{};
            SharedResource(const ctor_passkey &) : boost::enable_shared_from_this<SharedResource>() { printf("  Resource %p constructed\n", this); }
    };

    boost::weak_ptr<SharedResource> SharedResource::test_weak_ptr = boost::weak_ptr<SharedResource>();

}

#endif