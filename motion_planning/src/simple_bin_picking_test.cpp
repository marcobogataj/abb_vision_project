#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

const ros::Duration default_wait_duration{ 30 };

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

std::vector<double> marker2vector(visualization_msgs::MarkerArray::ConstPtr& cylinder_marker)
{
    std::vector<double> v;
    v.resize(9);

    v[0]=cylinder_marker->markers[0].pose.position.x;
    v[1]=cylinder_marker->markers[0].pose.position.y;
    v[2]=cylinder_marker->markers[0].pose.position.z;

    v[3]=cylinder_marker->markers[0].pose.orientation.x;
    v[4]=cylinder_marker->markers[0].pose.orientation.y;
    v[5]=cylinder_marker->markers[0].pose.orientation.z;
    v[6]=cylinder_marker->markers[0].pose.orientation.w;
    
    v[7]=cylinder_marker->markers[0].scale.x; //diameter
    v[8]=cylinder_marker->markers[0].scale.z; //height

    return v;
}

double GripperApertureConversion(double aperture_in_mm)
{
  double joint_value;
  //min: 0rad->0.085m, max: 0.8rad->0m
  joint_value = (0.085 - aperture_in_mm)/0.085 * 0.8; 
  return joint_value;
}

struct grasp_pose {
    geometry_msgs::Pose grasp_targetPose;
    geometry_msgs::Pose pregrasp_targetPose;
};

grasp_pose computeCylinderGrasp(visualization_msgs::MarkerArray::ConstPtr& cylinder_marker, double H, double D)
{
  grasp_pose grasp;
  geometry_msgs::Pose targetPose = cylinder_marker->markers[0].pose;
  tf2::Transform cylinderPose;
  tf2::fromMsg(targetPose,cylinderPose);
  tf2::Vector3 origin_x_axis(1,0,0), origin_y_axis(0,1,0), origin_z_axis(0,0,1);
  tf2::Vector3 tool0_x_axis, tool0_y_axis, tool0_z_axis;
  tf2::Vector3 cylinder_z_axis = cylinderPose.getBasis() * origin_z_axis;

  //TO DO: choose mode of grasping: vertical or horizontal
  std::string mode;

  //projection value used to choose between vertical and horizontal mode
  if (cylinder_z_axis.dot(origin_z_axis) >= 0){
      cylinder_z_axis = - cylinder_z_axis;
  }
    
  double pzvalue = cylinder_z_axis.dot(origin_z_axis) / (cylinder_z_axis.length() * origin_z_axis.length()) ; //value between -1 and 1
  double pyvalue = cylinder_z_axis.dot(origin_y_axis) / (cylinder_z_axis.length() * origin_y_axis.length()) ; //value between -1 and 1

  std::cout<<"abs pzvalue ="<<abs(pzvalue)<<std::endl;
  std::cout<<"pyvalue ="<<pyvalue<<std::endl;

  if (   (abs(pzvalue) > 0.97 && pyvalue > 0) //--> cylinder oriented opposite to robot
      || (abs(pzvalue) > 0.50 && pyvalue < 0)) //--> cylinder oriented to the robot
  { 
    mode = "vertical";
  }
  else 
  {
    mode = "horizontal";
  } 

  if (mode == "vertical")
  {
    ROS_INFO("Compute grasping in vertical mode...");
    tool0_z_axis = cylinder_z_axis; 

    tf2::Vector3 rob_to_cylinder(targetPose.position.x,targetPose.position.y,targetPose.position.z);

    //Get orthogonal component of rob_to_cylinder with respect to cylinder_z_axis
    //see https://en.wikipedia.org/wiki/Vector_projection
    tool0_x_axis = (rob_to_cylinder - rob_to_cylinder.dot(tool0_z_axis) * tool0_z_axis) * -1; 
    tool0_y_axis = tool0_x_axis.cross(tool0_z_axis) * -1;

    tool0_z_axis.normalize();

    grasp.grasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628-H/3); 
    grasp.grasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628-H/3);
    grasp.grasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628-H/3);

    grasp.pregrasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(0.1493+H); 
    grasp.pregrasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(0.1493+H);
    grasp.pregrasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(0.1493+H);
  }
  else
  {
    ROS_INFO("Compute grasping in horizontal mode...");
    
    if (cylinder_z_axis.dot(origin_y_axis) <= 0){
      cylinder_z_axis = - cylinder_z_axis;
    }

    tool0_x_axis = cylinder_z_axis; 

    tf2::Vector3 rob_to_cylinder(targetPose.position.x,targetPose.position.y,targetPose.position.z);

    //Get orthogonal component of rob_to_cylinder with respect to cylinder_z_axis
    //see https://en.wikipedia.org/wiki/Vector_projection
    tool0_y_axis = tool0_x_axis.cross(origin_z_axis);
    
    if (tool0_y_axis.dot(origin_x_axis) <= 0){
      tool0_y_axis = - tool0_y_axis;
    }

    tool0_z_axis = tool0_x_axis.cross(tool0_y_axis);

    tool0_z_axis.normalize();

    grasp.grasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628-D/4); 
    grasp.grasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628-D/4);
    grasp.grasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628-D/4);

    grasp.pregrasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(0.1493+D); 
    grasp.pregrasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(0.1493+D);
    grasp.pregrasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(0.1493+D);
  }

  //obtain quaternions for pose definiton
  tool0_z_axis.normalize();
  tool0_x_axis.normalize();
  tool0_y_axis.normalize();

  Eigen::Matrix3f tool0_basis;
  tool0_basis <<tool0_x_axis.getX(), tool0_y_axis.getX(), tool0_z_axis.getX(),
                tool0_x_axis.getY(), tool0_y_axis.getY(), tool0_z_axis.getY(),
                tool0_x_axis.getZ(), tool0_y_axis.getZ(), tool0_z_axis.getZ();

  Eigen::Quaternionf q(tool0_basis);

  targetPose.orientation.x = q.x();
  targetPose.orientation.y = q.y();
  targetPose.orientation.z = q.z();
  targetPose.orientation.w = q.w();

  //save to struct grasp_pose 
  grasp.grasp_targetPose.orientation = targetPose.orientation;
  grasp.pregrasp_targetPose.orientation = targetPose.orientation;

  return grasp;
}

void stampPosition(geometry_msgs::Pose Pose)
{
  static tf2_ros::StaticTransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "grasping_pose";
  transformStamped.transform.translation.x = Pose.position.x;
  transformStamped.transform.translation.y = Pose.position.y;
  transformStamped.transform.translation.z = Pose.position.z;

  transformStamped.transform.rotation.x = Pose.orientation.x;
  transformStamped.transform.rotation.y = Pose.orientation.y;
  transformStamped.transform.rotation.z = Pose.orientation.z;
  transformStamped.transform.rotation.w = Pose.orientation.w;

  transformStamped.header.stamp = ros::Time::now();
  tfb.sendTransform(transformStamped);
  ros::spinOnce;
} 

int main(int argc, char **argv)
{
    //creation of the node and named it
    ros::init(argc, argv, "simple_bin_picking_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    ros::Duration timeout(10);

    //create a move_group_interface object
    static const std::string PLANNING_GROUP_ARM = "irb_120";
    static const std::string PLANNING_GROUP_GRIPPER = "robotiq_gripper";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning arm_group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper_group(PLANNING_GROUP_GRIPPER);

    //Planning scene interface for collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", arm_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", arm_group.getEndEffectorLink().c_str());

    //Target position 
    geometry_msgs::Pose target_pose;
    visualization_msgs::MarkerArray::ConstPtr marker_array_msg(new visualization_msgs::MarkerArray);
    std::vector<double> v; //cylindwe params. [x,y,z,qx,qy,qz,qw,diameter,height]

    rviz_visual_tools::RvizVisualTools rviz_interface("base_link","/visualization_marker_array");

    // visualize the planning
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = arm_group.plan(my_plan);

    CHECK(ros::service::waitForService("/zivid_capture/zivid_capture_suggested",default_wait_duration));

    ROS_INFO("Ready to capture & pick");

    ROS_INFO("visualizing plan %s", success.val ? "":"FAILED");

    while(ros::ok())
    {
        std::string inputString;
        ROS_INFO("Type p to pick cylinder. Anything else to shutdown");
        std::getline(std::cin, inputString);

        // remove markers
        ROS_INFO("Clean all markers");
        rviz_interface.deleteAllMarkers();

        if(inputString == "p"){
            ROS_INFO("Start camera acquisition...");
            auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");
        }
        else{
            ros::shutdown();
        }

        ROS_INFO("Wait for cylinder identification...");


        marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array",nh,timeout);
        if (marker_array_msg == NULL)
        {
          ROS_INFO("No cylinder idientified!");
        }
        else
        {
          ROS_INFO("Cylinder idientified!");

          v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector

          double D = v[7];
          double H = v[8];

          std::cout<<"D: "<<D<<"mm "<<"H: "<<H<<"mm "<<std::endl;
          std::cout<<"Position [x,y,z]=["<<v[0]<<","<<v[1]<<","<<v[2]<<"]"<<std::endl;
          std::cout<<"Orientation [qx,qy,qz,qw]="<<v[3]<<","<<v[4]<<","<<v[5]<<","<<v[6]<<"]"<<std::endl;

          ROS_INFO("PICKING START!");

          // Compute grasping
          grasp_pose grasp = computeCylinderGrasp(marker_array_msg,H,D);

          //1 move the arm_group arm close to the target pose
          ROS_INFO("Moving to pre-grasp position...");
          target_pose = grasp.pregrasp_targetPose;
          stampPosition(target_pose);
          arm_group.setPoseTarget(target_pose);
          arm_group.move();

          ros::Duration(0.5).sleep(); 

          //2 move the arm_group arm to the target pose

          ROS_INFO("Moving to grasp position...");
          target_pose = grasp.grasp_targetPose;
          stampPosition(target_pose);
          arm_group.setPoseTarget(target_pose);
          arm_group.move();

          ros::Duration(0.5).sleep();

          //2 - Close gripper
          double joint_value = GripperApertureConversion(D-0.001);
          ROS_INFO("Close gripper to joint value %.4f",joint_value);
          gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);
          gripper_group.move();

          ros::Duration(0.5).sleep();

          //3 - Post-grasp movement
          ROS_INFO("Moving to post-grasp position...");
          target_pose = grasp.pregrasp_targetPose;
          stampPosition(target_pose);
          arm_group.setPoseTarget(target_pose);
          arm_group.move();

          ros::Duration(0.5).sleep();  

          //4 - Go home
          ROS_INFO("Going home...");
          arm_group.setJointValueTarget(arm_group.getNamedTargetValues("home"));
          arm_group.move();

          ros::Duration(0.5).sleep(); 

          //5 - Open Gripper
          ROS_INFO("Open gripper...");
          gripper_group.setJointValueTarget(gripper_group.getNamedTargetValues("open_gripper"));
          gripper_group.move();

          ros::Duration(3.0).sleep();

          ROS_INFO("PICKING COMPLETED!");

        }
    }


    ros::shutdown();
    return 0;

}