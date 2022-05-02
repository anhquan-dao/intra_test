#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <shm_transport/shm_topic.hpp>

#include <vector>
#include <opencv2/opencv.hpp>

#define MAP_SIZE 500*3
#define HZ 20

void imshowOccupancyGrid(std::string name, const cv::Mat &og_mat)
{
    cv::Mat img;
    og_mat.convertTo(img, CV_8UC1);
    // img.setTo(255, og_mat == 0);
    cv::imshow(name, og_mat);
    cv::waitKey(1);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map){
    cv::Mat raw_data_buffer;
    cv::Mat data_img_8u;
    // std::cout << map->info.height << std::endl;
    // std::cout << map->data.size() << std::endl;
    // std::cout << sizeof(map->data.front()) << std::endl;
    raw_data_buffer = cv::Mat(map->data, true).reshape(1, map->info.height);
    raw_data_buffer.convertTo(data_img_8u, CV_8UC1);
    // std::cout << "OK" << std::endl;
    imshowOccupancyGrid("Received map", data_img_8u);
}
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "shm_listener", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    shm_transport::Topic t(nh);
    shm_transport::Subscriber<nav_msgs::OccupancyGrid> s = t.subscribe("shm_map", 60, mapCallback);
    ros::spin();
    return 0;
}