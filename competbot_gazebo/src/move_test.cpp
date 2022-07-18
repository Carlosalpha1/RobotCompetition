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
#include <geometry_msgs/Twist.h>
#include <iostream>

enum states{
    MOVE_FORWARD_STATE = 0,
    CIRCLE_RIGHT_STATE = 1,
    CIRCLE_LEFT_STATE = 2,
    MOVE_BACKWARD_STATE = 3
};

class VelocityController
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;

    void move(float lx, float az)
    {
        geometry_msgs::Twist vel;
        vel.linear.x = lx;
        vel.angular.z = az;
        pub_.publish(vel);
    }

public:
    VelocityController(void)
    {
        pub_ = nh_.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 10);
    }

    void moveForward(void) { this->move(0.3, 0); }

    void moveBackward(void) { this->move(-0.3, 0); }
    
    void circleRight(void) { this->move(0.3, -0.3); }
    
    void circleLeft(void) { this->move(0.3, 0.3); }
};

class Timer
{
private:
    ros::Time timer_;

public:
    Timer()  {
        while (ros::Time::now().toSec() == 0) {
            ros::spinOnce();
        }
    }

    void start() { timer_ = ros::Time::now(); }
    
    void restart() { this->start(); }
    
    bool elapsedTime(float seconds)
    {
        return ((ros::Time::now().toSec() - timer_.toSec()) > seconds);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_test_node");

    VelocityController velocity_controller;
    Timer clock;
    
    int state = MOVE_FORWARD_STATE;
    ros::Rate loop_rate(30);
    
    clock.start();
    while(ros::ok()) {

        if (state == MOVE_FORWARD_STATE) {
            velocity_controller.moveForward();
            
            if (clock.elapsedTime(2)) {
                ROS_INFO("CIRCLE_RIGHT_STATE");
                state = CIRCLE_RIGHT_STATE;
                clock.restart();
            }
        }
        else if (state == CIRCLE_RIGHT_STATE) {
            velocity_controller.circleRight();

            if (clock.elapsedTime(8)) {
                ROS_INFO("MOVE_BACKWARD_STATE");
                state = MOVE_BACKWARD_STATE;
                clock.restart();
            }
        }
        else if (state == MOVE_BACKWARD_STATE) {
            velocity_controller.moveBackward();

            if (clock.elapsedTime(2)) {
                ROS_INFO("CIRCLE_LEFT_STATE");
                state = CIRCLE_LEFT_STATE;
                clock.restart();
            }
        }
        else if (state == CIRCLE_LEFT_STATE) {
            velocity_controller.circleLeft();

            if (clock.elapsedTime(4)) {
                ROS_INFO("MOVE_FORWARD_STATE");
                state = MOVE_FORWARD_STATE;
                clock.restart();
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
