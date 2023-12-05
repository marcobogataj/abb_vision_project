#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <pcl/filters/voxel_grid.h>

ros::Subscriber sub;
ros::Publisher pub;

typedef pcl::PointXYZ PointT;

//Problems!!!!!
/*Cannot use pcl::PointCloud<pcl::PointXYZRGBA> as receiving type when subscribing to /zivid_camera/points/xyzrgba
  While capturing&publishing is happening thanks to the zivid_camera driver, a warning appears on the subscriber node: "Failed to find match for field 'rgba' " 
  The saved pointcloud is correct for XYZ but without any data on RGBA fields. 
  This happens in the same manner using two different approches:
  
  1->Receiving sensor_msgs::PointCloud2 and converting it in two steps (first using pcl_conversions::toPCL, then pcl::fromPCLPointCloud2)

  2->Exploiting #include <pcl_ros/point_cloud.h> header, which offers seamless interoperability with non-PCL-using ROS nodes, allowing publishing and subscribing 
     pcl::PointCloud<PointT> objects as ROS messages, by hooking into the roscpp serialization infrastructure. 
     (Tutorial 3.2 from pcl_ros documentation http://wiki.ros.org/pcl_ros#Subscribing_to_point_clouds)

  The problem occurs only when trying to import XYZRGBA types, while subscribing to the XYZ Point Cloud topic using method 2) does not generate any problems.

  What's your advice on how to get RGBA point clouds? Ideally we want to convert them into the pcl::PointCloud<pcl::PointXYZRGBA> type, 
  as this is preferred when working with the pcl perception pipeline thanks to better indexing and other features.

  Could the answer be related on how you define function void ZividCamera::publishPointCloudXYZRGBA at line 572 of your zivid_camera.cpp found in the zivid_camera pkg?
*/

void cloud_callback(const pcl::PointCloud<PointT>::ConstPtr& input)
{ 
  printf ("Cloud: width = %d, height = %d\n", input->width, input->height);
  ROS_INFO("Point cloud received!");

  // Create the filtering object
  pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT> ());

  pcl::VoxelGrid<PointT> vox;
  vox.setInputCloud (input);
  vox.setLeafSize (0.001f, 0.001f, 0.001f);
  vox.filter (*output);

  pub.publish(*output); //send output message
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  sub = nh.subscribe<pcl::PointCloud<PointT>>("/zivid_camera/points/xyz", 1, cloud_callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<PointT>>("/pcl_processing/filtered/xyz", 1);

  // Spin
  ros::spin ();
}
