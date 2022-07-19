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

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <robot_competition_msgs/Pose.h>
#include <robot_competition_msgs/Poses.h>

#include <iostream>
#include <sstream>

class PosesCompute
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber blue_poses_sub_;
    ros::Subscriber ball_pose_sub_;
    ros::Subscriber cloud_sub_;
    
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_point_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_sub_;
    
    std::string working_frame_id_;
    std::string camera_topic_id_;

    robot_competition_msgs::Poses blue_team_poses_;
    robot_competition_msgs::Pose ball_pose_;

public:
    PosesCompute():
        working_frame_id_("/external_camera_tf/rgbd_camera_link"),
        camera_topic_id_("/external_camera/rgbd_camera/depth/points")
    {
        point_cloud_sub_ =
            new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, camera_topic_id_, 5);
        
        tf_point_cloud_sub_ =
            new tf::MessageFilter<sensor_msgs::PointCloud2> (*point_cloud_sub_, tf_listener_, working_frame_id_, 5);
        tf_point_cloud_sub_->registerCallback(boost::bind(&PosesCompute::cloudCallback, this, _1));
        
        blue_poses_sub_ = nh_.subscribe("/blue_team/poses", 1, &PosesCompute::blueposesCallback, this);
        ball_pose_sub_ = nh_.subscribe("/ball/pose", 1, &PosesCompute::ballposeCallback, this);
    }

    void blueposesCallback(const robot_competition_msgs::Poses::ConstPtr & msg)
    {
        blue_team_poses_ = *msg;
    }

    void ballposeCallback(const robot_competition_msgs::Pose::ConstPtr & msg)
    {
        ball_pose_ = *msg;
    }

    void publishTFPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pcrgb,
                       const robot_competition_msgs::Pose & pose, const std::string & object_frame)
    {
        auto point = pcrgb->at(pose.x, pose.y);

        std::stringstream ss;
        ss << object_frame << "/" << pose.id;

        tf::StampedTransform transform;
        transform.setOrigin(tf::Vector3(point.x, point.y, point.z));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

        transform.stamp_ = ros::Time::now();
        transform.frame_id_ = working_frame_id_;
        transform.child_frame_id_ = ss.str();

        try {
            tf_broadcaster_.sendTransform(transform);
        }
        catch(tf::TransformException& ex) {
            ROS_ERROR_STREAM("Transform error: " << ex.what());
            return;
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & cloud_in)
    {
        sensor_msgs::PointCloud2 cloud;
        
        try {
            pcl_ros::transformPointCloud(working_frame_id_, *cloud_in, cloud, tf_listener_);
        }
        catch(tf::TransformException & ex) {
            ROS_ERROR_STREAM("Transform error: " << ex.what());
            return;
        }

        if (blue_team_poses_.poses.size() == 0) return;
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(cloud, *pcrgb);

        for (auto pose : blue_team_poses_.poses) {
            publishTFPose(pcrgb, pose, "/blue_player");
        }

        publishTFPose(pcrgb, ball_pose_, "/ball");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poses_compute_node");
    ROS_INFO("[poses_compute_node]");
    PosesCompute pcl_compute;
    ros::spin();
    return 0;
}
