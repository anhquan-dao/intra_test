#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <shm_transport/shm_topic.hpp>

// #include <intra_test/shm_tp_lib_cpp.h>

#include <vector>

#define MSGLEN (1920 * 1080 * 3)
#define HZ (20)

std::string str(MSGLEN, '-');
char tmp[100];

int main(int argc, char ** argv) {
    ros::init(argc, argv, "shm_talker", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    shm_transport::Topic t(n);
    shm_transport::Publisher p = t.advertise< nav_msgs::OccupancyGrid >("shm_map", HZ, 3 * MSGLEN);

    std::vector<int8_t> map_vector;
    map_vector.assign(400, 0);

    nav_msgs::OccupancyGrid map;

    ros::Rate loop_rate(HZ);
    int count = 0;
    while (ros::ok()) {
        int len = snprintf(tmp, 100, "message # %d ...", count);
        memcpy(&str[0], tmp, len);

        std_msgs::String msg;
        msg.data = str;

        ROS_INFO("info: [%s]", tmp);
        map.data.assign(map_vector.begin(), map_vector.end());
        p.publish(map);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}
