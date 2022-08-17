// Copyright (C) 2022 Carlosalpha1

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <robot_competition_msgs/Pose.h>
#include <robot_competition_msgs/Poses.h>

#include <robot_competition_msgs/registry.h>

#include <iostream>
#include <vector>

ros::ServiceClient srv_reset_gazebo;

struct player_data
{
    std::string name;
    std::string team;
    int x_pos;
    int y_pos;
};

std::vector<player_data> players_registry;

void show_registry(void)
{
    std::cout << "--------------\n";
    for (auto registry : players_registry) {
        std::cout << "[" << registry.name << ", " << registry.team << "] "
                  << "(" << registry.x_pos << ", " << registry.y_pos << ")\n";
    }
    std::cout << "--------------\n";
}

bool register_service_callback(
    robot_competition_msgs::registry::Request & req,
    robot_competition_msgs::registry::Response & res)
{
    for (auto registry : players_registry) {
        if (registry.name == req.name && registry.team == req.team) {
            res.status = false;
            return true;
        }
    }
    res.status = true;
    
    struct player_data entry;
    entry.name = req.name;
    entry.team = req.team;
    entry.x_pos = req.pose.x;
    entry.y_pos = req.pose.y;
    players_registry.push_back(entry);

    show_registry();
    return true;
}

void ballpose_callback(const robot_competition_msgs::Pose::ConstPtr & msg)
{
    if (msg->x < 20) {
        std_srvs::Empty srv;
        if (srv_reset_gazebo.call(srv)) {
            ROS_INFO("Reseting World");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_competition_manager_node");
    ROS_INFO("[robot_competition_manager_node]");
    ros::NodeHandle nh;
    
    ros::ServiceServer service = nh.advertiseService("register_service", register_service_callback);

    // srv_reset_gazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    // ros::Subscriber ball_pose_sub = nh.subscribe("/ball/pose", 1, ballpose_callback);

    ros::spin();
    return 0;
}