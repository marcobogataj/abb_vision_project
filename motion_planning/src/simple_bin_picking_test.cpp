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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

int main(int argc, char **argv)
{
    //creation of the node and named it
    ros::init(argc, argv, "simple_bin_picking_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    //create a move_group_interface object
    static const std::string PLANNING_GROUP_ARM = "irb_120";
    static const std::string PLANNING_GROUP_GRIPPER = "robotiq_gripper";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning arm_group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper_group(PLANNING_GROUP_GRIPPER);


    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", arm_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", arm_group.getEndEffectorLink().c_str());

    //Target position (ASK FOR TARGET)
    geometry_msgs::Pose target_pose;
    visualization_msgs::MarkerArray::ConstPtr marker_array_msg(new visualization_msgs::MarkerArray);;
    std::vector<double> v; //cylindwe params. [x,y,z,qx,qy,qz,qw,diameter,height]

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

        if(inputString == "p"){
            ROS_INFO("Start camera acquisition...");
            auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");
        }
        else{
            ros::shutdown();
        }

        ROS_INFO("Wait for cylinder identification...");
        marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array",nh);
        ROS_INFO("Cylinder idientified!");

        v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector

        double D = v[7];
        double H = v[8];

        std::cout<<"D: "<<D<<"mm "<<"H: "<<H<<"mm "<<std::endl;
        std::cout<<"Position [x,y,z]=["<<v[0]<<","<<v[1]<<","<<v[2]<<"]"<<std::endl;
        std::cout<<"Orientation [qx,qy,qz,qw]="<<v[3]<<","<<v[4]<<","<<v[5]<<","<<v[6]<<"]"<<std::endl;

        ROS_INFO("PICKING START!");

        //1 move the arm_group arm close to the target pose
        ROS_INFO("Moving to pre-grasp position...");

        target_pose.position.x = v[0];
        target_pose.position.y = v[1];
        target_pose.position.z = v[2]+0.1493+std::sqrt(std::pow(H,2)+std::pow(D,2)); //keep awlays distant from grasped object

        tf2::Quaternion orientation;
        orientation.setRPY(M_PI , 0, M_PI / 2);
        target_pose.orientation = tf2::toMsg(orientation);

        arm_group.setPoseTarget(target_pose);
        
        arm_group.move();

        ros::Duration(1.0).sleep(); 

        //2 move the arm_group arm to the target pose

        ROS_INFO("Moving to grasp position...");

        target_pose.position.z = v[2]+0.1628-H/2;; //0.09 is the distance from tool_0 to the palm

        orientation.setRPY(M_PI , 0, M_PI / 2);
        target_pose.orientation = tf2::toMsg(orientation);

        arm_group.setPoseTarget(target_pose);
        
        arm_group.move();

        ros::Duration(1.0).sleep();

        //2 - Close gripper
        double joint_value = GripperApertureConversion(D-0.005);
        ROS_INFO("Close gripper to joint value %.4f",joint_value);

        gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);

        gripper_group.move();

        ros::Duration(1.0).sleep(); 

        //3 - Go home
        ROS_INFO("Going home...");

        arm_group.setJointValueTarget(arm_group.getNamedTargetValues("home"));

        arm_group.move();

        ros::Duration(1.0).sleep(); 

        //4 - Open Gripper
        ROS_INFO("Open gripper...");

        gripper_group.setJointValueTarget(gripper_group.getNamedTargetValues("open_gripper"));

        gripper_group.move();

        ros::Duration(3.0).sleep();

        ROS_INFO("PICKING COMPLETED!");
    }


    ros::shutdown();
    return 0;

}