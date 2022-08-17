#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <robot_competition_msgs/registry.h>

#include <iostream>

void usage(char **argv)
{
    ROS_ERROR("Usage: %s player team", argv[0]);
}

class PlayerTask
{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_vel_;
    
    ros::Subscriber odom_sub_;
    
    ros::ServiceClient register_client_;

public:
    PlayerTask(const std::string & player, const std::string & team)
    {
        registry(player, team);
        mainloop();
    }

    void registry(const std::string & player, const std::string & team)
    {
        ROS_INFO("Registring %s from %s", player.c_str(), team.c_str());

        this->register_client_ = nh_.serviceClient<robot_competition_msgs::registry>("register_service");
        
        nav_msgs::OdometryConstPtr initial_pos;
        do{
            initial_pos = ros::topic::waitForMessage<nav_msgs::Odometry>(
            "/robot/odom", this->nh_, ros::Duration(2));
        } while (initial_pos == nullptr);

        robot_competition_msgs::registry player_info;
        player_info.request.name = player;
        player_info.request.team = team;
        player_info.request.pose.x = initial_pos->pose.pose.position.x;
        player_info.request.pose.y = initial_pos->pose.pose.position.y;

        if (!this->register_client_.call(player_info)) {
            ROS_ERROR("Registry Service Error");
        }

        if (player_info.response.status == false) {
            ROS_ERROR("Request not permitted!");
        }
    }

    void mainloop(void)
    {
        return;
    }
};

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage(argv);
        std::exit(1);
    }
    ros::init(argc, argv, "player_node");

    PlayerTask ptask(argv[1], argv[2]);

    ros::spin();
    return 0;
}