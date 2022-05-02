#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <shm_transport/shm_topic.hpp>

#include <vector>
#include <memory>
#include <chrono>
#include <limits>
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <opencv2/opencv.hpp>



namespace intraprocess_test{ 
    namespace py = boost::python;
    namespace np = boost::python::numpy;

    template<class M> void callback(const boost::shared_ptr< const M > &input_data);
    template<> void callback<nav_msgs::OccupancyGrid>(const boost::shared_ptr<const nav_msgs::OccupancyGrid> &input_data){
        ROS_INFO_STREAM("Hello");
    }
    template<> void callback<sensor_msgs::Image>(const boost::shared_ptr<const sensor_msgs::Image> &input_data){
        ROS_INFO_STREAM("Hello");
    }

    template<class M> class SHMSubscriberCPP{
        // typedef void (*Func)(const boost::shared_ptr< const M > &);

        public:
            SHMSubscriberCPP() = delete;
            SHMSubscriberCPP(std::string topic_name)
            {
                topic_name_ = topic_name;
                sub = topic.subscribe(topic_name_, 60, callback<M>);
                // if(topic_type == "nav_msgs::OccupancyGrid"){
                //     sub = topic.subscribe(topic_name_, 60, callback<nav_msgs::OccupancyGrid>);
                // }
            }
            ~SHMSubscriberCPP() = default;

            // void setCallback(Func fp){
            //     sub = topic.subscribe(topic_name_, 60, fp);
            // }

            py::object image = py::import("sensor_msgs.msg/Image");
        
        private:
            ros::NodeHandle private_nh;
            shm_transport::Topic topic{private_nh};
            std::string topic_name_;
            shm_transport::Subscriber<M> sub;
      
    };

    uint8_t updateMap(const uint8_t first_cell, const uint8_t second_cell){
        return second_cell;
    }

    template<class M> void pypublish_helper(py::object object, const shm_transport::Publisher &pub);

    template<> void pypublish_helper<nav_msgs::OccupancyGrid>(py::object object, const shm_transport::Publisher &pub){
        nav_msgs::OccupancyGrid cpp_occupancygrid;
        
        // Extracting metadata of the Python OccupancyGrid object
        cpp_occupancygrid.header.frame_id = py::extract<std::string>(object.attr("header").attr("frame_id"));
        
        ros::Time msg_time(py::extract<float>(object.attr("header").attr("stamp").attr("to_sec")()));
        cpp_occupancygrid.header.stamp = msg_time;

        cpp_occupancygrid.info.width = py::extract<float>(object.attr("info").attr("width"));
        cpp_occupancygrid.info.height = py::extract<float>(object.attr("info").attr("height"));
        cpp_occupancygrid.info.resolution = py::extract<float>(object.attr("info").attr("resolution"));

        // Extracting map data of the Python OccupancyGrid object
        // Two popular methods proposed on forums are
        // FOR statement and std::vector::assign() whose execution time
        // increases with time.
        // INSTEAD std::copy_n is used which does not
        // take into account of the last stl_interator of boost::python::list
        // for (int i = 0; i < py::len(obj_list); ++i)
        //     cpp_map.insert(cpp_map.begin()+i, py::extract<uint8_t>(obj_list[i]));
        // AND
        // cpp_map.assign(py::stl_input_iterator<uint8_t>(obj_list),
        //                py::stl_input_iterator<uint8_t>()); 
        // AND
        // std::transform(dummy_map.begin(), dummy_map.end(),
        //                py::stl_input_iterator<uint8_t>(obj_list),
        //                cpp_map.begin(), updateMap);

        py::list obj_list = py::extract<py::list>(object.attr("data")); 

        std::vector<uint8_t> cpp_map;
        std::vector<uint8_t> dummy_map;

        uint32_t map_flat_size = cpp_occupancygrid.info.width * cpp_occupancygrid.info.height;
        cpp_map.assign(map_flat_size,0);
        dummy_map.assign(map_flat_size, 0);  

        std::copy_n(py::stl_input_iterator<uint8_t>(obj_list), map_flat_size, cpp_map.begin());

        cpp_occupancygrid.data.assign(cpp_map.begin(), cpp_map.end());

        pub.publish(cpp_occupancygrid);
        
    }

    template<class M> class SHMPublisherCPP{
        public:
            SHMPublisherCPP() = delete;
            SHMPublisherCPP(std::string topic_name)
            {   
                uint32_t msg_size = 1920 * 1080 * 3;
                ROS_INFO_STREAM(sizeof(M));
                pub = topic.advertise<nav_msgs::OccupancyGrid>(topic_name, 50, (uint32_t)(3*msg_size));
                ROS_INFO_STREAM("SHMTransport Publisher initialized");
            }
            ~SHMPublisherCPP() = default;

            void pypublish(boost::python::object object){
                auto start = std::chrono::high_resolution_clock::now();
                auto duration = start.time_since_epoch();
                double millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                // ROS_INFO_STREAM("Hello from Publisher");
                pypublish_helper<M>(object, pub);
            }
        
        private:
            ros::NodeHandle private_nh;
            shm_transport::Topic topic{private_nh};
            shm_transport::Publisher pub;

        friend void pypublish_helper<nav_msgs::OccupancyGrid>(py::object object, const shm_transport::Publisher &pub);
            
            
    };
}
