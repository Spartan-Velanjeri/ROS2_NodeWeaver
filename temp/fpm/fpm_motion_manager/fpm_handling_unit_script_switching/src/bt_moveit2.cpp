#include "moveit.cpp"
//#include "releasebrakes.cpp"
//#include "playpausestop.cpp"
#include "robot_scene.cpp"
#include "load_script.cpp"
#include "exec_script.cpp"
#include "on_robot_ur.cpp"
#include "loadprogram_ur.cpp"
#include "moveit_control.cpp"
#include "remove_scene.cpp"
#include "iocondition.cpp"
#include "moveit_joints.cpp"
//#include "points_client.cpp"

#include "moveit_cluster.cpp"
//FPM motion server clients
//#include "configured_pose_client.cpp"
//#include "move_lift_absolute_client.cpp"
//#include "move_absolute_client.cpp"
//Data service clients
#include "load_points_client.cpp"
#include "points_load.cpp"
#include "update_index.cpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>  // std::min
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <string>
#include <vector>
#define DEFAULT_BT_XML "/home/avp1le/ros2_ws/src/ur_btree/ur_bts/bt_xml/urbt_xml.xml"

using namespace BT;

//std::shared_ptr<rclcpp::Node> node;
//std::vector<POI> pois;
//std::atomic<unsigned int> poi_index{0};
//double points[100][3];
//geometry_msgs::msg::PoseArray posearray;
//geometry_msgs::msg::Pose p;
//int32_t i = 0;
//int32_t points_count = 0;

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    
    auto nh = rclcpp::Node::make_shared("task_switching");
    nh->declare_parameter("bt_xml");
    std::string bt_xml;
    nh->get_parameter("bt_xml", bt_xml);
    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());


    
    //node = rclcpp::Node::make_shared("behavior_tree");
    //pois = load_pois_from_file(
      //  ament_index_cpp::get_package_share_directory("fpm_handling_unit_script_switching") + "/config/cluster_points.yaml");
    
    
    RCLCPP_INFO(nh->get_logger(), "Registering Nodes");
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;
    
    
    factory.registerNodeType<LoadScript>("loadscript");
    factory.registerNodeType<Moveit>("moveit");
    factory.registerNodeType<ExecScript>("execscript");
    factory.registerNodeType<Play>("playur");
    factory.registerNodeType<LoadProgram>("loadur");
    factory.registerNodeType<MoveitControl>("moveitcontrol");
    factory.registerNodeType<LoadScene>("loadscene");
    factory.registerNodeType<RemoveScene>("removescene");
    factory.registerNodeType<Iostatecondition>("iostatecondition");
    factory.registerNodeType<MoveitJoints>("moveitjoints");
    factory.registerNodeType<GetPoi>("getpoi");
    factory.registerNodeType<UpdatePoi>("updatepoi");
    factory.registerNodeType<MoveitCluster>("moveitcluster");
    factory.registerNodeType<LoadPoints>("loadpoints");
    //factory.registerNodeType<MoveAbsoluteClient>("moveabsolute");
    //factory.registerNodeType<ConfiguredPoseClient>("configuredpose");
    //factory.registerNodeType<MoveLiftAbsoluteClient>("moveliftabsolute");
    //factory.registerNodeType<PointsClient>("pointsload");
    RCLCPP_INFO(nh->get_logger(), "Registered Nodes");
    
    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    
    // Create a logger
    auto blackboard = BT::Blackboard::create();
    auto tree = factory.createTreeFromFile(bt_xml, blackboard);
   
    //blackboard->set("points_count_bb", points_count);
    //blackboard->set("points", posearray);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the total count is: ", points_count_bb);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the total count is: %d", points_count);
    int32_t point_index = 0;
    blackboard->set("point_index", point_index);

    BT::PublisherZMQ publisher_zmq(tree);
    StdCoutLogger logger_cout(tree);
  
    
    RCLCPP_INFO(nh->get_logger(), "Running BT");
    //NodeStatus status = NodeStatus::RUNNING;
    //tree.tickRoot();
    tree.tickRootWhileRunning();
    //while (rclcpp::ok()) {
        //tree.tickRoot();
        //rclcpp::spin_some(nh);
    //}
    return 0;
}