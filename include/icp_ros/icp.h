/*
 *  Copyright (c) 2023, CAI-Lab, Inc.
 *  All rights reserved. 
 */

/* Author: OkDoky */

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "std_srvs/SetBool.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
//for point_cloud::fromROSMsg
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

//#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread/mutex.hpp>

class ScanMatcherICPNode
{
    public: 
        ScanMatcherICPNode();
        ~ScanMatcherICPNode();
        void matrixAsTransfrom(const Eigen::Matrix4f &in_mat,  tf::Transform& out_transform);
        pcl::KdTree<pcl::PointXYZ>::Ptr getTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb);
        void mapCallback(const nav_msgs::OccupancyGrid& msg);
        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
        bool getTransform(tf::StampedTransform &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp);
        void updateParams();

    private:
        ros::NodeHandle n;
        ros::NodeHandle private_nh_;
        ros::Publisher pub_output_scan_matched;
        ros::Publisher pub_output_scan_unmatched;

        ros::Subscriber subMap;
        ros::Subscriber subScan;

        ros::Time last_processed_scan;
        ros::Time paramsWereUpdated;
        ros::Time begin_; 
        double update_time_;

        std::string BASE_LASER_FRAME;
        std::string ODOM_FRAME;
        std::string BASE_FRAME;
        std::string MAP_FRAME;

        //these should be parameters defines how good the match has to be to create a candidate for publishing a pose
        //defines how much distance deviation from amcl to icp pose is needed to make us publish a pose
        double DIST_THRESHOLD;
        double DIST_UPPER_THRESHOLD;
        // accept only scans one second old or younger
        double TIME_THRESHOLD;
        double UPDATE_TIME;

        double ICP_INLIER_DIST;

        double ICP_NUM_ITER;

        double SCAN_RATE;

        bool map_used;
        bool scan_used;
        bool scan_transformed_used;

        bool use_sim_time;

        bool m_monitor_debug_flag;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_xyz;
        pcl::KdTree<pcl::PointXYZ>::Ptr mapTree;
        boost::shared_ptr< sensor_msgs::PointCloud2> output_cloud;
        boost::shared_ptr< sensor_msgs::PointCloud2> scan_cloud;

        boost::mutex scan_callback_mutex;

        laser_geometry::LaserProjection *projector_;
        tf::TransformListener *listener_;
        tf::TransformListener listener;
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::PointCloud2 cloud2transformed_2;
        sensor_msgs::PointCloud2 cloud2transformed_3;
};
