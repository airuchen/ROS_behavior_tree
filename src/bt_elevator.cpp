#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>

using namespace BT;

typedef enum {IDLE, UP, DOWN, WAIT} CURRENT_STATE;

struct ElevatorState
{
	CURRENT_STATE current_state;
	int current_floor;
	int next_goal;
	int goal_temp[10] = {0};
};

// namespace BT
// {
// 	template <> inline ElevatorState convertFromString(StringView str)
// 	{
// 		auto part
// 	}
// }

class Initial : public SyncActionNode
{
	public:
		Initial(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

		NodeStatus tick() override
		{
			ElevatorState e_state;
			e_state.current_state = IDLE;
			e_state.current_floor = 1;
			e_state.next_goal = 4;

			ROS_INFO("Initialized, welcome aboard.");
			setOutput("state", e_state);
			return NodeStatus::SUCCESS;
		}

		static PortsList providedPorts()
		{
			return {OutputPort<ElevatorState>("state")};
		}
};

class WaitForGoal : public SyncActionNode
{
	public:
		WaitForGoal(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

		NodeStatus tick() override
		{
			Optional<ElevatorState> state = getInput<ElevatorState>("state");
			std::cout << "enter your desired floor" << std::endl;
			int floor;
			std::cin >> floor;
			state->next_goal = floor;
			setOutput("state", state);
			return NodeStatus::SUCCESS;
		}

		static PortsList providedPorts()
		{
			return {OutputPort<ElevatorState>("state"), InputPort<ElevatorState>("state")};
		}
};

class GoFloor : public SyncActionNode
{
	public:
		GoFloor(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

		NodeStatus tick() override
		{
			Optional<ElevatorState> state = getInput<ElevatorState>("state");

			std::cout << "Toward " << state->next_goal << " floor." << std::endl;
			return NodeStatus::SUCCESS;
		}

		static PortsList providedPorts()
		{
			return {InputPort<ElevatorState>("state")};
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

	factory.registerNodeType<Initial>("Initial");
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
