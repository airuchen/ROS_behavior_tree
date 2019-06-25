#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

class SaySomething : public SyncActionNode
{
public:
	SaySomething(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

	static PortsList providedPorts()
	{
		return {InputPort<std::string>("message")};
	}

	NodeStatus tick() override
	{
		Optional<std::string> msg = getInput<std::string>("message");
		if(!msg)
		{
			throw BT::RuntimeError("missing required input [message]:", msg.error());
		}

		ROS_INFO("%s", msg.value().c_str());
		std::cout << "Robot says: " << msg.value() << std::endl;
		return NodeStatus::SUCCESS;
	}
};


class ThinkWhatToSay: public SyncActionNode
{
public:
	ThinkWhatToSay(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

	static PortsList providedPorts()
	{
		return { OutputPort<std::string>("text") };
	}

	NodeStatus tick() override
	{
		setOutput("text", "The answer is yes");
		return NodeStatus::SUCCESS;
	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "bt_tutorial_2");

	ros::NodeHandle nh("~");
	std::string xml_filename;
	nh.getParam("file", xml_filename);
	ROS_INFO("Loading XML : %s", xml_filename.c_str());

	BehaviorTreeFactory factory;

	factory.registerNodeType<SaySomething>("SaySomething");
	factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
	PortsList say_something_ports = {InputPort<std::string> ("message")};

	auto tree = factory.createTreeFromFile(xml_filename);

	NodeStatus status = NodeStatus::RUNNING;
	while (ros::ok() && status == NodeStatus::RUNNING)
	{
		tree.root_node->executeTick();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return 0;
}
