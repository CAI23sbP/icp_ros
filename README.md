# icp_ros
saperate scan as mathed with map or not



## 1.0 How to run

```bash
roslaunch icp_ros icp_ns.launch
```

if you don't want to use name space of node, remove ns="" inside launch file



## 2.0 I/O

#### 2.1 icp_ros_node

##### 2.1.1 Subscribed Topics

scan (sensor_msgs/LaserScan)

​	The laser scan to compare with map

map (nav_msgs/OccupancyGrid)

​	default base to compare with scan data



##### 2.1.2 Published Topics

scan_matched (sensor_msgs/PointCloud2)

​	laser scan data matched with map

scan_unmatched (sensor_msgs/PointCloud2)

​	laser scan data unmatched with map data (unknown obstacles)



##### 2.1.3 Parameters

odom_frame (string, default: "odom")

​	robot odom frame

base_laser_frame (string, default: "base_laser_link")

​	robot base laser link frame

map_frame (string, default: "map")

​	map frame

base_frame (string, default: "base_link")

​	robot base link

update_time (double, default: 0.0)

​	expected update cycle time, minimum cycle time, if set 0.0, than update when get scan

use_sim_time (boolen, default: false)

​	use sim time

time_threshold (double, default: 1.0)

​	scan time threshold	

dist_upper_threshold (double, default: 1.0)

​	distance from map to scan point minimum threshold to saperate matched or unmatched point	

icp_inlier_dist (double, default: 0.1)

​	icp distance form pointcloud to map point

icp_num_iter (double, default: 100)

​	max icp iteration count

scan_rate (double, default: 1)

​	expected scan rate

icp_mode (boolen, default: false)

​	use icp mode to set pose

monitoring_mode (boolen, default: true)

