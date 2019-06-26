/*
 * In tutorial 5, we want to create __hierarchical__ behavior trees.
 * This can be achieved easily defining multiple trees in the XML including one into the other.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

struct Pose2D
{
	double x, y, theta;
};

namespace BT
{
	template <> inline Pose2D convertFromString(StringView str)
	{
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
}

class MoveBaseAction : public AsyncActionNode
{
	public:
		MoveBaseAction(const std::string& name, const NodeConfiguration& config) : AsyncActionNode(name, config) {}

		static PortsList providedPorts()
		{
			return {InputPort<Pose2D>("goal")};
		}

		NodeStatus tick() override
		{
			Pose2D goal;
			if (!getInput<Pose2D>("goal", goal))
			{
				throw RuntimeError("missing required input [goal]");
			}

			printf("[MoveBase: STARTED]. goal: x=%.f y=%.1f theta=%.2f\n", goal.x, goal.y, goal.theta);

			_halt_requested.store(false);
			int count = 0;

			while (!_halt_requested && count++ < 25)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}

			std::cout << "[ MoveBase: FINISHED ]" << std::endl;
			return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
		}

		void halt() override
		{
			_halt_requested.store(true);
		}

	private:
		std::atomic_bool _halt_requested;

};


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
			if (!msg)
			{
				throw RuntimeError("missing required input [message]", msg.error() );
			}

			std::cout << "Robot says: " << msg.value() << std::endl;
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
	factory.registerNodeType<MoveBaseAction>("MoveBase");


	auto tree = factory.createTreeFromFile(xml_filename);

	// This logger prints state changes on console
	StdCoutLogger logger_cout(tree);

	NodeStatus status = NodeStatus::RUNNING;

	while(ros::ok() && status == NodeStatus::RUNNING)
	{
			status = tree.root_node->executeTick();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	std::cout << "------------------" << std::endl;
	tree.blackboard_stack[0]->debugMessage();
	std::cout << "------------------" << std::endl;
	tree.blackboard_stack[1]->debugMessage();
	std::cout << "------------------" << std::endl;

	return 0;
}
