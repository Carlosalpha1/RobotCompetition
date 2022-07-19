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
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <robot_competition_msgs/Pose.h>
#include <robot_competition_msgs/Poses.h>

#include <iostream>
#include <vector>

static const std::string cv_wnd = "Camera Controller";

class CameraController
{
private:
    ros::NodeHandle nh_;
    ros::Publisher blue_team_pub_;
    ros::Publisher ball_pub_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    

public:
    CameraController()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("/external_camera/rgbd_camera/rgb/image_raw", 1,
            &CameraController::imageCallback, this);
        
        blue_team_pub_ = nh_.advertise<robot_competition_msgs::Poses>("/blue_team/poses", 1);
        ball_pub_ = nh_.advertise<robot_competition_msgs::Pose>("/ball/pose", 1);
    }

    ~CameraController()
    {
    }

    void imageCallback(const sensor_msgs::ImageConstPtr & msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        namespace enc = sensor_msgs::image_encodings;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception & e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->detection(cv_ptr);
    }

    void detection(cv_bridge::CvImagePtr & cv_ptr)
    {
        cv::Mat image = cv_ptr->image;

        cv::Mat blue_team_mask = detectBlueTeam(image);
        cv::Mat ball_mask = detectBall(image);
        
        auto blue_poses = findPoses(blue_team_mask);
        auto ball_pose = findPoses(ball_mask);

        if (blue_poses.poses.size() > 0) {
            blue_team_pub_.publish(blue_poses);
        }

        if (ball_pose.poses.size() > 0) {
            ball_pub_.publish(ball_pose.poses[0]);
        }

        // cv::imshow(cv_wnd, ball_mask);
        // cv::waitKey(3);
    }

    robot_competition_msgs::Poses findPoses(const cv::Mat & mask)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
        robot_competition_msgs::Poses poses;
        for( int i = 0; i < contours.size(); i++) {
            robot_competition_msgs::Pose pose;
            cv::Moments mu = cv::moments(contours[i], false);
            pose.id = i+1;
            pose.x = mu.m10/mu.m00;
            pose.y = mu.m01/mu.m00;
            poses.poses.push_back(pose);
        }

        return poses;
    }

    cv::Mat imgThreshold(const cv::Mat & src, const cv::Scalar & hsv_low, const cv::Scalar & hsv_high)
    {
        cv::Mat hsv_image;
        cv::cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat img_threshold;
        cv::inRange(hsv_image, hsv_low, hsv_high, img_threshold);
        
        cv::erode(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        return img_threshold;
    }

    cv::Mat detectBall(cv::Mat src)
    {
        return imgThreshold(src, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255));
    }

    cv::Mat detectBlueTeam(cv::Mat src)
    {
        return imgThreshold(src, cv::Scalar(110, 100, 100), cv::Scalar(130, 255, 255));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_controller_node");
    ROS_INFO("[camera_controller_node]");
    CameraController camera_controller;
    ros::spin();
    return 0;
}
