/*
 * In tutorial 4, The next example shows the difference between a `SequenceNode` and a 'ReactiveSequence`.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

struct Pose2D
{
	double x;
	double y;
	double theta;
};

namespace BT
{
	template <> inline Pose2D convertFromString(StringView str)
	{
		printf("Converting string: \"%s\"\n", str.data());
		// real numbers separated by semicolons
		auto parts = splitString(str, ';');
		if (parts.size() !=3)
		{
			throw RuntimeError("invalid input");
		}
		else{
			Pose2D output;
			output.x = convertFromString<double>(parts[0]);
			output.y = convertFromString<double>(parts[1]);
			output.theta = convertFromString<double>(parts[2]);
			return output;
		}
	}
} // end namespace BT


class MoveBaseAction : public AsyncActionNode
{
public:
	MoveBaseAction(const std::string& name, const NodeConfiguration& config) : AsyncActionNode(name, config){}

	static PortsList providedPorts()
	{
		return {InputPort<Pose2D>("goal")};
	}

	NodeStatus tick() override
	// This overloaded method is used to stop the execution of this node.
	{
		Pose2D goal;
		if ( !getInput<Pose2D>("goal", goal))
		{
			throw RuntimeError("missing required input [goal]");
		}

		printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n", goal.x, goal.y, goal.theta);

		_halt_requested.store(false);

		int count = 0;
		// Pretend that "computing" takes 250milliseconds.
		while (!_halt_requested && count++ < 25)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		std::cout << "[MoveBase: FINISHED ]" << std::endl;
		return _halt_requested? NodeStatus::FAILURE : NodeStatus::SUCCESS;
	}

	void halt() override
	{
		_halt_requested.store(true);
	}

private:
	std::atomic_bool _halt_requested;
};


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "bt_tutorial_2");

	ros::NodeHandle nh("~");
	std::string xml_filename;
	nh.getParam("file", xml_filename);
	ROS_INFO("Loading XML : %s", xml_filename.c_str());

	BehaviorTreeFactory factory;

	factory.registerNodeType<MoveBaseAction>("MoveBase");

	auto tree = factory.createTreeFromFile(xml_filename);

	NodeStatus status = NodeStatus::RUNNING;

	std::cout << "\n --- 1st executeTick() ---" << std::endl;
	status = tree.root_node->executeTick();

	std::this_thread::sleep_for(std::chrono::milliseconds(150));
	std::cout << "\n --- 2nd executeTick() ---" << std::endl;
	status = tree.root_node->executeTick();

	std::this_thread::sleep_for(std::chrono::milliseconds(150));
	std::cout << "\n --- 3rd executeTick() ---" << std::endl;
	status = tree.root_node->executeTick();

	std::cout << std::endl;

	return 0;
}
