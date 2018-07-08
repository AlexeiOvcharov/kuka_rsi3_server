#include "kuka_rsi3/CommunicationClient.hpp"

int main(int argc, char ** argv)
{

	std::string nodeName = "communication_client";

    /* ROS initialization */
    ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    std::string armName;

    /* configuration */
    std::string address;
    int port;
    double communicationFrequency;
    nh.param<std::string> ("communication_address", address, "127.0.0.1");
    nh.param<std::string> ("arm_name", armName, "kuka_arm");
    nh.param<int> ("communication_port", port, 2002);
    nh.param<double> ("rsi_communication_frequency", communicationFrequency, 250);


	CommunicationClient client(nh, address, port);

	client.setCommunicationFrequency(communicationFrequency);
    client.initializeArm(armName);

    ros::spin();
}