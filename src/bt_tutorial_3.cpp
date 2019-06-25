/*
 * In tutorial 3, you will learn how to use self-defined date types  in behavior tree.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

struct Position2D
{
	double x;
	double y;
};

namespace BT
{
	template <> inline Position2D convertFromString(StringView str)
	{
		printf("Converting string: \"%s\"\n", str.data());
		// real numbers separated by semicolons
		auto parts = splitString(str, ';');
		if (parts.size() !=2)
		{
			throw RuntimeError("invalid input");
		}
		else{
			Position2D output;
			output.x = convertFromString<double>(parts[0]);
			output.y = convertFromString<double>(parts[1]);
			return output;
		}
	}
} // end namespace BT

class CalculateGoal : public SyncActionNode
{
	public:
		CalculateGoal(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

		static PortsList providedPorts()
		{
			return { OutputPort<Position2D>("goal")};
		}

		NodeStatus tick() override
		{
			Position2D mygoal = {1.1, 2.3};
			setOutput<Position2D>("goal", mygoal);
			return NodeStatus::SUCCESS;
		}
};


class PrintTarget : public SyncActionNode
{
public:
	PrintTarget(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

	static PortsList providedPorts()
	{
		const char* description = "Simply print the goal on console...";
		return { InputPort<Position2D>("target", description)};
	}

	NodeStatus tick() override
	{
		auto res = getInput<Position2D>("target");
		if (!res)
		{
			throw RuntimeError("error readin port [target]:", res.error());
		}
		Position2D target = res.value();
		printf("Target position: [ %.1f, %.1f ]\n", target.x, target.y);
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

	factory.registerNodeType<CalculateGoal>("CalculateGoal");
	factory.registerNodeType<PrintTarget>("PrintTarget");

	auto tree = factory.createTreeFromFile(xml_filename);

	NodeStatus status = NodeStatus::RUNNING;
	while (ros::ok() && status == NodeStatus::RUNNING)
	{
		tree.root_node->executeTick();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return 0;
}
