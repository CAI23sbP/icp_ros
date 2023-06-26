/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** Modified by OkDoky */

#include "icp_ros/icp.h"

static double normalize(double z){
    return atan2(sin(z),cos(z));
}

ScanMatcherICPNode::ScanMatcherICPNode(): private_nh_("~"), listener_(&listener)
{
    updateParams();
    output_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
    scan_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
    
    map_used = true;
    scan_used = true;
    scan_transformed_used = true;
    m_monitor_debug_flag = true;
    
    projector_ = new laser_geometry::LaserProjection();
    
    pub_output_scan_matched = n.advertise<sensor_msgs::PointCloud2> ("scan_matched", 1);
    pub_output_scan_unmatched = n.advertise<sensor_msgs::PointCloud2> ("scan_unmatched", 1);
    
    subMap = n.subscribe("map", 1, &ScanMatcherICPNode::mapCallback, this);
    subScan = n.subscribe("scan", 1, &ScanMatcherICPNode::scanCallback, this);


    try{
        listener_->waitForTransform(BASE_FRAME, MAP_FRAME, ros::Time(0), ros::Duration(30.0));
        listener_->waitForTransform(BASE_LASER_FRAME, MAP_FRAME, ros::Time(0), ros::Duration(30.0));
    } catch (...){
        ROS_WARN("[ICP] fail to init wait for transform");
    }
    last_processed_scan = ros::Time::now();

    ROS_WARN("[ICP] end initialize");
}

ScanMatcherICPNode::~ScanMatcherICPNode(void)
{
}

void ScanMatcherICPNode::matrixAsTransfrom(const Eigen::Matrix4f &in_mat,  tf::Transform& out_transform){

    double mv[12];

    mv[0] = in_mat (0, 0) ;
    mv[4] = in_mat (0, 1);
    mv[8] = in_mat (0, 2);
    mv[1] = in_mat (1, 0) ;
    mv[5] = in_mat (1, 1);
    mv[9] = in_mat (1, 2);
    mv[2] = in_mat (2, 0) ;
    mv[6] = in_mat (2, 1);
    mv[10] = in_mat (2, 2);

    tf::Matrix3x3 basis;
    basis.setFromOpenGLSubMatrix(mv);
    tf::Vector3 origin(in_mat (0, 3),in_mat (1, 3),in_mat (2, 3));

    ROS_DEBUG("[ICP] origin %f %f %f", origin.x(), origin.y(), origin.z());

    out_transform = tf::Transform(basis, origin);
}

pcl::KdTree<pcl::PointXYZ>::Ptr ScanMatcherICPNode::getTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb){

    pcl::KdTree<pcl::PointXYZ>::Ptr tree;
    tree.reset (new pcl::KdTreeFLANN<pcl::PointXYZ>);

    tree->setInputCloud(cloudb);
    return tree;
}

void ScanMatcherICPNode::mapCallback(const nav_msgs::OccupancyGrid& msg){

    ROS_WARN("I heard frame_id: [%s]", msg.header.frame_id.c_str());

    float resolution = msg.info.resolution;
    float width = msg.info.width;
    float height = msg.info.height;

    float posx = msg.info.origin.position.x;
    float posy = msg.info.origin.position.y;

    cloud_xyz = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());

    //cloud_xyz->width    = 100; // 100
    cloud_xyz->height   = 1;
    cloud_xyz->is_dense = false;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = MAP_FRAME;
    cloud_xyz->header = pcl_conversions::toPCL(header);

    pcl::PointXYZ point_xyz;

    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            //@TODO
            if (msg.data[x + y * width] == 100) // 100 = occupied, -1 = free, 0 = unoccupied
            {
                point_xyz.x = (.5f + x) * resolution + posx;
                point_xyz.y = (.5f + y) * resolution + posy;
                point_xyz.z = 0;
                cloud_xyz->points.push_back(point_xyz);
            }
        }
    cloud_xyz->width = cloud_xyz->points.size();

    mapTree = getTree(cloud_xyz);

    pcl::toROSMsg(*cloud_xyz, *output_cloud);
    ROS_WARN("Publishing PointXYZ cloud with %ld points in frame %s", cloud_xyz->points.size(),output_cloud->header.frame_id.c_str());

    map_used = true;
}

void ScanMatcherICPNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    // updateParams();
    ros::Time start_cycle_time = ros::Time::now();
    if(!map_used)
    {
        ROS_WARN("[ICP] Waiting for map to be published");
        return;
    }

    ros::Time scan_in_time = scan_in->header.stamp;
    ros::Time time_received = ros::Time::now();

    if(scan_in_time - last_processed_scan < ros::Duration(1.0f / SCAN_RATE) )    
    {
        ROS_DEBUG("[ICP] Waiting for scan, last %f , this %f", last_processed_scan.toSec() ,scan_in_time.toSec());
        return;    
    }
    //projector_.transformLaserScanToPointCloud(BASE_FRAME,*scan_in,cloud,listener_);
    if(!this->scan_callback_mutex.try_lock())
        return;
    ros::Duration scan_age = ros::Time::now() - scan_in_time;

    //check if we want to accept this scan, if its older than 1 sec, drop it
    if(!use_sim_time)
    {
        if(scan_age.toSec() > TIME_THRESHOLD)
        {
            ROS_WARN("[ICP] scan date id delayed (%f seconds, %f threshold) scan time: %f , now %f", scan_age.toSec(), TIME_THRESHOLD, scan_in_time.toSec(),ros::Time::now().toSec() );
            this->scan_callback_mutex.unlock();

            return;
        }
    }

    ros::Time now_ = ros::Time::now();
    ros::Duration diff_time = now_ - begin_;
    if(diff_time.sec >= UPDATE_TIME) 
    {
        begin_ = ros::Time::now();

        tf::StampedTransform base_at_laser;
        if (!getTransform(base_at_laser, ODOM_FRAME, BASE_FRAME, scan_in_time))
        {
            ROS_WARN("[ICP] Did not get base pose at laser scan time");
            this->scan_callback_mutex.unlock();
            return;
        }

        sensor_msgs::PointCloud cloud;
        sensor_msgs::PointCloud cloudInMap;

        projector_->projectLaser(*scan_in, cloud); // Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2. Project a single laser scan from a linear array into a 3D point cloud. The generated cloud will be in the same frame as the original laser scan.
        scan_used = false;
        bool gotTransform = false;

        if(!listener_->waitForTransform(MAP_FRAME, cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.05)))
        {
            this->scan_callback_mutex.unlock();
            ROS_WARN("[ICP] SnapMapICP no map to cloud transform found MUTEX UNLOCKED");
            return;
        }

        if(!listener_->waitForTransform(MAP_FRAME, BASE_FRAME, cloud.header.stamp, ros::Duration(0.05)))
        {
            this->scan_callback_mutex.unlock();
            ROS_WARN("[ICP] SnapMapICP no map to base transform found MUTEX UNLOCKED");
            return;
        }
        while(!gotTransform && (ros::ok()))
        {
            try
            {
                gotTransform = true;
                listener_->transformPointCloud(MAP_FRAME, cloud, cloudInMap); // Void tf::TransformListener::transformPointCloud(const std::__cxx11::string &target_frame, const sensor_msgs::PointCloud &pcin, sensor_msgs::PointCloud &pcout) const
            }
            catch (...)
            {
                gotTransform = false;
                ROS_WARN("[ICP] Didn't get transform in A");
            }
        }
        for(size_t k =0; k < cloudInMap.points.size(); k++)
        {
            cloudInMap.points[k].z = 0;
        }

        gotTransform = false;
        tf::StampedTransform oldPose;
        while(!gotTransform && (ros::ok()))
        {
            try
            {
                gotTransform = true;
                listener_->lookupTransform(MAP_FRAME, BASE_FRAME, cloud.header.stamp , oldPose);
            }
            catch (tf::TransformException ex)
            {
                gotTransform = false;
                ROS_WARN("[ICP] Didn't get transform in B");
            }
        }
        if(map_used && gotTransform)
        {
            sensor_msgs::convertPointCloudToPointCloud2(cloudInMap, cloud2);
            scan_used = true;

            //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
            //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
            reg.setTransformationEpsilon(1e-6);
            
            // Note: adjust this based on the size of your datasets
            reg.setMaxCorrespondenceDistance(DIST_UPPER_THRESHOLD); // Set the maximum distance between two correspondences (src<->tgt) to 10cm
            reg.setMaximumIterations(ICP_NUM_ITER);
            // Set the point representation

            //ros::Time bef = ros::Time::now();

            PointCloudT::Ptr pcl_XYZ_cloud(new PointCloudT());
            PointCloudT::Ptr input_scan_cloud(new PointCloudT());

            pcl::fromROSMsg(*output_cloud, *pcl_XYZ_cloud); // void pcl::fromROSMsg<pcl::PointXYZ>(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
            pcl::fromROSMsg(cloud2, *input_scan_cloud); 

            reg.setInputSource(input_scan_cloud); // A, changed from reg.setInputCloud(input_scan_cloud) since deprecated
            reg.setInputTarget(pcl_XYZ_cloud); // B,  pcl_XYZ_cloud = map cloud from scan data

            PointCloudT unused;
            reg.align(unused);  // Transformed source data(=A)

            const Eigen::Matrix4f &transf = reg.getFinalTransformation(); // Matrix4, Get the final transformation matrix estimated by the registration method.
            tf::Transform t;
            matrixAsTransfrom(transf,t);

            scan_transformed_used = false;
            PointCloudT transformedCloud;
            PointCloudT unmatchedCloud;
            pcl::transformPointCloud (*input_scan_cloud, transformedCloud, reg.getFinalTransformation());

            // void pcl::transformPointCloud<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, const Eigen::Matrix4f &transform, bool copy_all_fields = true)

            double inlier_perc = 0;
            
            // count inliers
            std::vector<int> nn_indices(1);
            std::vector<float> nn_sqr_dists(1);

            size_t numinliers = 0;

            for (size_t k = 0; k < transformedCloud.points.size(); ++k )
            {
                if (mapTree->radiusSearch(transformedCloud.points[k], ICP_INLIER_DIST, nn_indices, nn_sqr_dists, 1) != 0)
                    numinliers += 1;
                else unmatchedCloud.push_back(transformedCloud.points[k]);
            }

            if (transformedCloud.points.size() > 0)
            {
                ROS_DEBUG("[ICP] Inliers in dist %f: %zu of %zu percentage %f", ICP_INLIER_DIST, numinliers, transformedCloud.points.size(), (double) numinliers / (double) transformedCloud.points.size());
                inlier_perc = (double) numinliers / (double) transformedCloud.points.size();
            }
            

            last_processed_scan = scan_in_time;
            pcl::toROSMsg(transformedCloud, cloud2transformed_2);
            pcl::toROSMsg(unmatchedCloud, cloud2transformed_3);
            std_msgs::Header header;
            header.stamp = ros::Time(0.0);
            header.frame_id = MAP_FRAME;

            cloud2transformed_3.header = header;
            cloud2transformed_3.height = 1;
            cloud2transformed_3.point_step = 16;
            cloud2transformed_3.width = cloud2transformed_3.data.size() / 16;
            cloud2transformed_3.row_step = cloud2transformed_3.data.size();

            scan_transformed_used = true;

            double dist = sqrt((t.getOrigin().x() * t.getOrigin().x()) + (t.getOrigin().y() * t.getOrigin().y()));
            double angleDist = t.getRotation().getAngle();
            tf::Vector3 rotAxis  = t.getRotation().getAxis();
            t =  t * oldPose;

            tf::StampedTransform base_after_icp;
            if(!getTransform(base_after_icp, ODOM_FRAME, BASE_FRAME, ros::Time(0)))
            {
                ROS_WARN("[ICP] Did not get base pose at now");
                this->scan_callback_mutex.unlock();

                return;
            }
            else
            {
                tf::Transform rel = base_at_laser.inverseTimes(base_after_icp);
                ROS_DEBUG("[ICP] relative motion of robot while doing icp: %f [cm] %f [deg]", rel.getOrigin().length(), rel.getRotation().getAngle() * 180 / M_PI);
                t= t * rel;
            }
            

            if(m_monitor_debug_flag == true){
                pub_output_scan_matched.publish(cloud2transformed_2);
                pub_output_scan_unmatched.publish(cloud2transformed_3);
            }
            
        }
    }
    this->scan_callback_mutex.unlock();
    ros::Time end_cycle_time = ros::Time::now();
    double cycle_time = (end_cycle_time - start_cycle_time).toSec();
    try{
        ROS_DEBUG("[ICP] icp cycle rate : %.8f",1.0/cycle_time);
    } catch (...){
        ROS_DEBUG("[ICP] icp cycle time : %.8f", cycle_time);
    }
    
}

bool ScanMatcherICPNode::getTransform(tf::StampedTransform &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp){

    bool gotTransform = false;

    ros::Time before = ros::Time::now();
    if (!listener_->waitForTransform(parent_frame, child_frame, stamp, ros::Duration(0.5)))
    {
        ROS_WARN("[ICP] Didn't get transform %s %s IN c at %f", parent_frame.c_str(), child_frame.c_str(), stamp.toSec());
        return false;
    }

    try
    {
        gotTransform = true;
        listener_->lookupTransform(parent_frame,child_frame,stamp , trans);
    }
    catch (tf::TransformException ex)
    {
        gotTransform = false;
        ROS_WARN("[ICP] Didn't get transform %s %s IN B", parent_frame.c_str(), child_frame.c_str());
    }

    return gotTransform;
}

void ScanMatcherICPNode::updateParams(){
    
    paramsWereUpdated = ros::Time::now();
    // nh.param<std::string>("default_param", default_param, "default_value");
    private_nh_.param<std::string>("odom_frame", ODOM_FRAME, "odom");
    private_nh_.param<std::string>("base_laser_frame", BASE_LASER_FRAME, "base_laser_link");
    private_nh_.param<std::string>("map_frame", MAP_FRAME, "map");
    private_nh_.param<std::string>("base_frame", BASE_FRAME, "base_link");
    private_nh_.param<double>("update_time", UPDATE_TIME, 0.0);  // Updated time for spreading particles
  
    private_nh_.param<bool>("use_sim_time", use_sim_time, false);
    private_nh_.param<double>("time_threshold", TIME_THRESHOLD, 1);
    private_nh_.param<double>("dist_upper_threshold", DIST_UPPER_THRESHOLD, 1.0);
    private_nh_.param<double>("icp_inlier_dist", ICP_INLIER_DIST, 0.1);
    private_nh_.param<double>("icp_num_iter", ICP_NUM_ITER, 100);
    private_nh_.param<double>("scan_rate", SCAN_RATE, 1);
    
    if (SCAN_RATE < .001)
        SCAN_RATE  = .001;

    ROS_WARN("[ICP] odom frame : %s",ODOM_FRAME.c_str());
    ROS_WARN("[ICP] base laser frame : %s", BASE_LASER_FRAME.c_str());
    ROS_WARN("[ICP] map frame : %s", MAP_FRAME.c_str());
    ROS_WARN("[ICP] base frame : %s", BASE_FRAME.c_str());
    ROS_WARN("[ICP] update time : %.4f", UPDATE_TIME);
    std::stringstream ss;
    ss << std::boolalpha << use_sim_time;
    std::string use_sim_time_ = ss.str();
    ROS_WARN("[ICP] use sim time : %s", use_sim_time_.c_str());
    ROS_WARN("[ICP] time threshold : %.4f", TIME_THRESHOLD);
    ROS_WARN("[ICP] dist upper threshold : %.4f", DIST_UPPER_THRESHOLD);
    ROS_WARN("[ICP] icp inlier dist : %.4f", ICP_INLIER_DIST);
    ROS_WARN("[ICP] icp num iter : %.4f", ICP_NUM_ITER);
    ROS_WARN("[ICP] scan rate : %.4f", SCAN_RATE);
}
        
int main(int argc, char** argv){

    ros::init(argc, argv, "scan_matcher_ICP");
    tf:
    ScanMatcherICPNode icp;
    ros::Rate loop_rate(1);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
