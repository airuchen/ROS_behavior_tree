#include "bt_sequence.h"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bt_turtle");

  ros::NodeHandle nh("~");
  std::string xml_filename;
  nh.getParam("file", xml_filename);
  ROS_INFO("Loading XML : %s", xml_filename.c_str());


  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

	NodeBuilder builder_MoveForward = [](const std::string& name, const NodeConfiguration& config)
	{
		ros::NodeHandle node;
		ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
		return std::make_unique<MoveForward>(name, config, pub);
	};

	factory.registerBuilder<MoveForward>("MoveForward", builder_MoveForward);

  factory.registerNodeType<Turn>("Turn");

	GripperInterface gripper;
	factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
	factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));


  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    status = tree.root_node->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
		if (status==NodeStatus::SUCCESS)
			status = NodeStatus::RUNNING;
  }
  return 0;
}

