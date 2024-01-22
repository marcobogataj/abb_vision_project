#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/time.h>

//specific processing libraries
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>

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
  ROS_INFO("Point cloud received!");
  printf ("Input cloud: width = %d, height = %d\n", input->width, input->height);

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT> ());

  // START of Filtering (1-Voxelization,2-Passthrough,3-Outliers ...)
  // https://stackoverflow.com/questions/66103202/how-to-downsample-a-point-cloud-while-maintaining-its-order-pcl-c

  // 1-Voxelization
  pcl::VoxelGrid<PointT> vox;
  vox.setInputCloud (input);
  vox.setLeafSize (0.001f, 0.001f, 0.001f);
  vox.filter (*output);
  *cloud = *output; //save to raw cloud for a new filter

  // 2-PASSTHROUGH
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-50, 50);
  pass.filter (*output);
  *cloud = *output; //save to raw cloud for a new filter

  // 3-STATISTICAL OUTLIER
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (100);
  sor.setStddevMulThresh (1);
  sor.filter (*output); 

  printf ("Filtered cloud: width = %d, height = %d\n", output->width, output->height);
  // END of Filtering

  // Declare extraction pointer
  pcl::ExtractIndices<PointT> extract;

  // START OF Segmentation 
  // 2-CLUSTER EXTRACTION 
  ROS_INFO("Start cluster extraction...");
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (1); 
  ec.setMinClusterSize (300);
  ec.setMaxClusterSize (1800);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::cout<< cluster_indices.size() <<" clusters found"<<std::endl;

  //Declare cloud vector, add each cluster to a new cloud vector slot
  //In visualization, generate a unique color for each cluster vector element 
  //and add to the visualizer

  // Extract points and copy them to clouds vector -> v_segment_clouds
  std::vector< pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > v_segment_clouds;
  pcl::PointCloud<PointT>::Ptr curr_segment_cloud;
  pcl::PointCloud<PointT>::Ptr res_cloud (new pcl::PointCloud<PointT>), res_cloud_temp (new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr idx; 

  //define objects for centroid extraction
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator <pcl::PointXYZ> > v_centroids;
  Eigen::Vector4f centroid;
  pcl::PointXYZ centroid_point;
 
  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //std::cout<<"Saving cluster "<< j+1 <<" to point cloud cluster vector..."<<std::endl;
    v_segment_clouds.push_back(cloud_cluster);

    //std::cout<<"Compute and save cluster centroid..."<<std::endl;
    pcl::compute3DCentroid(*cloud_cluster, centroid); 

    //First three elements of centroid are x,y,x. Last is 1 to allow 4x4 matrix transformations
    //Save centroid as PointXYZ
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];

    v_centroids.push_back(centroid_point); //add to vector of centroids

    std::cout<<"Cluster "<< j+1 <<" size: "<< cloud_cluster->size() <<std::endl;
    j++;
  }

  // END of SEGMENTATION

  // FIT PRIMITIVE MODELS on clusters
  // Fit cylinder

  // Idea: 
  // use pcl::SACSegmentationFromNormals with setModelType (pcl::SACMODEL_CYLINDER) to estimate
  // cylinder coefficients point_on_axis (𝑐), axis_direction (𝑣), radius (R).
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  std::vector<pcl::ModelCoefficients> v_coefficients_cylinder;
  pcl::ModelCoefficients::Ptr coefficients_cyl_temp (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
  std::vector<float> h; //cylinder heigh estimation
  std::vector<int> cyl_found; //1-> cyl found, 0->cyl not found
  pcl::PointCloud<PointT>::Ptr line_proj (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr merged_cloud_cylinder (new pcl::PointCloud<PointT>);


  //ITERATE CLUSTERS
  ROS_INFO("Start cylindrical fitting...");

  int i=0;
  for(auto& segment : v_segment_clouds)
  {
    // Estimate point normals
    ne.setSearchMethod (tree2);
    ne.setInputCloud (segment);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg2.setOptimizeCoefficients (true);
    seg2.setModelType (pcl::SACMODEL_CYLINDER);
    seg2.setMethodType (pcl::SAC_RANSAC);
    seg2.setNormalDistanceWeight (0.2);
    seg2.setMaxIterations (5000);
    seg2.setDistanceThreshold (0.015);
    seg2.setRadiusLimits (0.015,0.085);
    seg2.setInputCloud (segment);
    seg2.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg2.segment (*inliers_cylinder, *coefficients_cyl_temp);
    std::cerr << "Compute cylinder coefficients for cluster "<<i+1<< "..." << std::endl;

    extract.setInputCloud (segment);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);

    int size_cyl = cloud_cylinder->width*cloud_cylinder->height;

    if ((cloud_cylinder->points.empty ()) || (cloud_cylinder->size() < segment->size()*0.70)) //0.7 parameter to be tuned!
    {
      cyl_found.push_back(0); //not found 

      std::cout << "Can't find the cylindrical component." << std::endl<<std::endl;
    }
    else
    { 
      cyl_found.push_back(1); //found

      // Display cylinder info
      std::cout << "Point on axis  ->(" << coefficients_cyl_temp->values[0] << ", " 
                                        << coefficients_cyl_temp->values[1] << ", "
                                        << coefficients_cyl_temp->values[2] << ") "<<std::endl<< 
                  "Axis direction ->("  << coefficients_cyl_temp->values[3] << ", "
                                        << coefficients_cyl_temp->values[4] << ", "
                                        << coefficients_cyl_temp->values[5] << ") "<<std::endl<< 
                  "Radius = "           << coefficients_cyl_temp->values[6] <<std::endl;

      std::cout<< "Cylinder inlier points:" << cloud_cylinder->size() <<std::endl<<std::endl;

      // To estimate cylinder height: https://math.stackexchange.com/questions/3324579/sorting-collinear-points-on-a-3d-line
      // 1-> Project cylinder inliers onto the cylinder axis 𝑣. (https://pcl.readthedocs.io/projects/tutorials/en/latest/project_inliers.html)
      // 2-> Select first and last points in the cylinder point cloud (ONLY with ordered point clouds file types)

      pcl::ModelCoefficients::Ptr coefficients_line (new pcl::ModelCoefficients ());
      coefficients_line->values.resize (6);
      
      for(int k=0; k<=5; k++) coefficients_line->values[k] = coefficients_cyl_temp->values[k];

      std::cout << "Point on axis ->("  << coefficients_line->values[0] << ", " 
                                        << coefficients_line->values[1] << ", "
                                        << coefficients_line->values[2] << ") "<<std::endl<< 
                  "Axis direction ->("  << coefficients_line->values[3] << ", "
                                        << coefficients_line->values[4] << ", "
                                        << coefficients_line->values[5] << ") "<<std::endl;

      pcl::ProjectInliers<PointT> proj;
      proj.setModelType (pcl::SACMODEL_LINE);
      proj.setInputCloud (cloud_cylinder);
      proj.setModelCoefficients (coefficients_line);
      proj.filter (*line_proj);

      std::cout<< "Cylinder inlier points:" << line_proj->size() <<std::endl;

      //compute height by computing the segment connecting first and last points of the cylinder inliers
      //since the point cloud is organized and saved in a ordered fashion, these results in the two extremes on the cyl axis.
      std::cout<<"Pmin="<<line_proj->points[0]<<std::endl;
      std::cout<<"Pmax="<<line_proj->points[line_proj->size()-1]<<std::endl<<std::endl;

      h.push_back(pcl::euclideanDistance(line_proj->points[0],line_proj->points[line_proj->size()-1])); //cylinder height estimation

      coefficients_cyl_temp->values[0]=line_proj->points[0].x;
      coefficients_cyl_temp->values[1]=line_proj->points[0].y;
      coefficients_cyl_temp->values[2]=line_proj->points[0].z;
      coefficients_cyl_temp->values[3]=line_proj->points[line_proj->size()-1].x - line_proj->points[0].x;
      coefficients_cyl_temp->values[4]=line_proj->points[line_proj->size()-1].y - line_proj->points[0].y;
      coefficients_cyl_temp->values[5]=line_proj->points[line_proj->size()-1].z - line_proj->points[0].z;

      v_coefficients_cylinder.push_back(*coefficients_cyl_temp);
      *merged_cloud_cylinder += *cloud_cylinder;
    }
  }
  // END OF Segmentation 

  pub.publish(*merged_cloud_cylinder); //send output message

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // capture()

  // Create a ROS subscriber for the input point cloud
  sub = nh.subscribe<pcl::PointCloud<PointT>>("/zivid_camera/points/xyz", 1, cloud_callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<PointT>>("/pcl_processing/filtered/xyz", 1);

  // processing()

  // Spin
  ros::spin ();
}
