#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class MoveForward : public BT::SyncActionNode {
public:
  MoveForward(const std::string &name, const BT::NodeConfiguration &config, ros::Publisher pub)
      : BT::SyncActionNode(name, config), _pub(pub) {}

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }

  virtual BT::NodeStatus tick() override;
private:
	ros::Publisher _pub;
};

class Turn : public BT::SyncActionNode {
public:
  Turn(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }

  virtual BT::NodeStatus tick() override;

private:
	ros::NodeHandle node;
	ros::Publisher _pub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
};


class GripperInterface
{
public:
  GripperInterface() : _open(true) {}

	BT::NodeStatus open(){
		_open = true;
		ROS_INFO("GripperInterface::open");
		return BT::NodeStatus::SUCCESS;
	}

	BT::NodeStatus close(){
		ROS_INFO("GripperInterface::close");
		_open = false;
		return BT::NodeStatus::SUCCESS;
	}

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }

private:
	bool _open; // shared information
};

// BT::NodeStatus CheckBattery()
// {
// 	ROS_INFO("Battery : OK");
// 	return BT::NodeStatus::SUCCESS;
// }
