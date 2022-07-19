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

#include <iostream>

ros::ServiceClient srv_reset_gazebo;

void ballpose_callback(const robot_competition_msgs::Pose::ConstPtr & msg)
{
    if (msg->x < 20) {
        std_srvs::Empty srv;
        if (srv_reset_gazebo.call(srv)) {
            ROS_INFO("Reseting World");
        };
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_competition_manager_node");
    ROS_INFO("[robot_competition_manager_node]");
    ros::NodeHandle nh;
    
    srv_reset_gazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    ros::Subscriber ball_pose_sub = nh.subscribe("/ball/pose", 1, ballpose_callback);

    ros::spin();
    return 0;
}