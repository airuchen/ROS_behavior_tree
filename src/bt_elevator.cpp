#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>

using namespace BT;

struct ElevatorState
{
	enum current_direction{idle, up, down};
	int next_goal;
	int goal_temp[10] = {0};
};

class WaitForGoal : public SyncActionNode
{
	public:
		WaitForGoal(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

		NodeStatus tick() override
		{
			std::string floor;
			std::cout << "enter your desired floor" << std::endl;
			std::cin >> floor;
			setOutput("target", floor);
			return NodeStatus::SUCCESS;
		}

		static PortsList providedPorts()
		{
			return {OutputPort<int>("target")};
		}
};

class GoFloor : public SyncActionNode
{
	public:
		GoFloor(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

		NodeStatus tick() override
		{
			Optional<std::string> msg = getInput<std::string>("goal");

			if(!msg)
			{
				throw BT::RuntimeError("missing required input [goal]: ", msg.error());
			}

			std::cout << "Toward " << msg.value() << " floor." << std::endl;
			return NodeStatus::SUCCESS;
		}

		static PortsList providedPorts()
		{
			return {InputPort<int>("goal")};
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "elevator_simulator");

	ros::NodeHandle nh("~");
	std::string xml_filename;
	nh.getParam("file", xml_filename);
	ROS_INFO("Loading XML: %s", xml_filename.c_str());

	BehaviorTreeFactory factory;

	factory.registerNodeType<WaitForGoal>("WaitForGoal");
	factory.registerNodeType<GoFloor>("GoFloor");

	auto tree = factory.createTreeFromFile(xml_filename);

	StdCoutLogger logger_cout(tree);

	NodeStatus status = NodeStatus::RUNNING;

	while(ros::ok() && status == NodeStatus::RUNNING)
	{
		status = tree.root_node->executeTick();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return 0;
}
