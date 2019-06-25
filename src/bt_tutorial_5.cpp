/*
 * In tutorial 5, we want to create __hierarchical__ behavior trees.
 * This can be achieved easily defining multiple trees in the XML including one into the other.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ros/ros.h>
using namespace BT;

NodeStatus IsDoorOpen()
{
	// std::cout << "[ Door: OPEN]" << std::endl;
	// return BT::NodeStatus::SUCCESS;
	std::cout << "[ Door: CLOSED]" << std::endl;
	return BT::NodeStatus::FAILURE;
}

class OpenDoor
{
public:
	OpenDoor(): _open(true){}

	NodeStatus open(){
		_open = true;
		std::cout << "Open door" << std::endl;
		return NodeStatus::FAILURE; // The door cannot be opened
		// return NodeStatus::SUCCESS; // Succeed opening the door
	}
private:
	bool _open;
};


class PassThroughDoor
{
public:
	PassThroughDoor(): _pass(true){}

	NodeStatus pass(){
		_pass = true;
		std::cout << "Pass door" << std::endl;
		return NodeStatus::SUCCESS;
	}
private:
	bool _pass;
};


class PassThroughWindow
{
public:
	PassThroughWindow(): _pass(true){}

	NodeStatus pass(){
		_pass = true;
		std::cout << "Pass window" << std::endl;
		return NodeStatus::FAILURE; // The robot is not able to pass the window
		// return NodeStatus::SUCCESS;
	}
private:
	bool _pass;
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "bt_tutorial_2");

	ros::NodeHandle nh("~");
	std::string xml_filename;
	nh.getParam("file", xml_filename);
	ROS_INFO("Loading XML : %s", xml_filename.c_str());

	BehaviorTreeFactory factory;

	factory.registerSimpleCondition("IsDoorOpen", std::bind(IsDoorOpen));
	PassThroughDoor passdoor;
	factory.registerSimpleAction("PassThroughDoor", std::bind(&PassThroughDoor::pass, &passdoor));

	PassThroughWindow passwindow;
	factory.registerSimpleAction("PassThroughWindow", std::bind(&PassThroughWindow::pass, &passwindow));

	OpenDoor opendoor;
	factory.registerSimpleAction("OpenDoor", std::bind(&OpenDoor::open, &opendoor));

	auto tree = factory.createTreeFromFile(xml_filename);

	// This logger prints state changes on console
	StdCoutLogger logger_cout(tree);
	// This logger saves state changes on file
	// FileLogger logger_file(tree.root_node, "bt_trace.fbl");
	// This logger stores the execution time of each node
	// MinitraceLogger logger_minitrace(tree.root_node, "bt_trace.json");

	printTreeRecursively(tree.root_node);

	while(ros::ok())
	{
		NodeStatus status = NodeStatus::RUNNING;
		while ( status == NodeStatus::RUNNING)
		{
			status = tree.root_node->executeTick();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}
	return 0;
}
