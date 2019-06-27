#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

struct Point3D
{
	double x, y, z;
};


class MyLegacyMoveTo
{
	public:
		bool go(Point3D goal)
		{
			printf("Going to : %f %f %f\n", goal.x, goal.y, goal.z);
			return true;
		}
};


namespace BT
{
	template <> inline Point3D convertFromString(StringView str)
	{
		auto parts = splitString(str, ';');
		if (parts.size() !=3)
		{
			throw RuntimeError("invalid input");
		}
		else{
			Point3D output;
			output.x = convertFromString<double>(parts[0]);
			output.y = convertFromString<double>(parts[1]);
			output.z = convertFromString<double>(parts[2]);
			return output;
		}
	}
}


int main(int argc, char ** argv)
{
	using namespace BT;

	ros::init(argc, argv, "bt_tutorial_7");

	ros::NodeHandle nh("~");
	std::string xml_filename;
	nh.getParam("file", xml_filename);
	ROS_INFO("Loading XML : %s", xml_filename.c_str());

	MyLegacyMoveTo move_to;

	auto MoveToWrapperWithLambda = [&move_to](TreeNode& parent_node)->NodeStatus
	{
		Point3D goal;
		parent_node.getInput("goal", goal);

		bool res = move_to.go(goal);
		return res? NodeStatus::SUCCESS : NodeStatus::FAILURE;
	};

	BehaviorTreeFactory factory;

	PortsList ports = {BT::InputPort<Point3D>("goal")};
	factory.registerSimpleAction("MoveTo", MoveToWrapperWithLambda, ports);

	auto tree = factory.createTreeFromFile(xml_filename);

	// This logger prints state changes on console
	StdCoutLogger logger_cout(tree);

	NodeStatus status = NodeStatus::RUNNING;

	while(ros::ok() && status == NodeStatus::RUNNING)
	{
			status = tree.root_node->executeTick();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	return 0;
}
