#ifndef COMMUNICATION_CLIENT
#define COMMUNICATION_CLIENT

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <iostream>

// ROS
#include <ros/ros.h>

#include <kuka_rsi3/CommandComunication.hpp>
#include <kuka_rsi3/TrajectoryGenerator.hpp>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <brics_actuator/JointPositions.h>
#include <trajectory_msgs/JointTrajectory.h>


typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
class CommunicationClient
{
    public:
        CommunicationClient(ros::NodeHandle & nh, std::string address, short int port);
        ~CommunicationClient();

        void initializeArm(std::string armName);

        // For big values of frequency
        void setCommunicationFrequency(double freq);

    private:

        // Communicztion function by TCP/IP
        void communication(std::string address, short int port);

        bool trajectoryLineInterpolation();

        // Callbacks
        void jointPositionCallback(const brics_actuator::JointPositions & msg);
        void followJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal);

        ros::NodeHandle node;

        // /kuka_arm/position_command subscriber
        ros::Subscriber positionCommandSubscriber;

        // Follow joint trajectory action server
        Server * trajectoryActionServer;

        // TCP/IP client for set the data
        CommandClient * client;
        io_service service;

        // Joint position movement variables
        JointVal currJntAng, desiredJntAng;

        // For check that joint position message is receive
        bool receive;

        // Trajectory configuration
        JointVal vel, accel;

        double communicationFrequency;

        boost::thread * communicationThread;

        trajectory_msgs::JointTrajectory actualTrajectory;

        bool trajectoryIsActive;

        control_msgs::FollowJointTrajectoryResult resultOfTrajExecution;

};

#endif