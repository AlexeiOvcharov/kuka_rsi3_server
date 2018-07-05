#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <iostream>

// ROS
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>

#include "kuka_rsi3/CommandComunication.hpp"
#include "kuka_rsi3/TrajectoryGenerator.hpp"

#define OFFSET_A1 0
#define OFFSET_A2 -90
#define OFFSET_A3 90
#define OFFSET_A4 0
#define OFFSET_A5 0
#define OFFSET_A6 0
#define TIME_STEP 0.004

JointVal currJntAng, desiredJntAng, ang;
bool receive = false;

void jointPositionCallback(const brics_actuator::JointPositions & msg) {
    receive = true;
    for (size_t i = 0; i < msg.positions.size(); ++i) {
        if (msg.positions[i].joint_uri == "joint_a1")
            desiredJntAng(0) = msg.positions[i].value * 180/M_PI - OFFSET_A1;
        if (msg.positions[i].joint_uri == "joint_a2")
            desiredJntAng(1) = msg.positions[i].value * 180/M_PI - OFFSET_A2;
        if (msg.positions[i].joint_uri == "joint_a3")
            desiredJntAng(2) = msg.positions[i].value * 180/M_PI - OFFSET_A3;
        if (msg.positions[i].joint_uri == "joint_a4")
            desiredJntAng(3) = msg.positions[i].value * 180/M_PI - OFFSET_A4;
        if (msg.positions[i].joint_uri == "joint_a5")
            desiredJntAng(4) = msg.positions[i].value * 180/M_PI - OFFSET_A5;
        if (msg.positions[i].joint_uri == "joint_a6")
            desiredJntAng(5) = msg.positions[i].value * 180/M_PI - OFFSET_A6;
    }
}

void displayJV(JointVal & jv, std::string title)
{
    std::cout << title << " (" << jv(0);
    for (size_t i = 1; i < DOF; ++i)
        std::cout << ", " << jv(i);
    std::cout << ")" << std::endl;
}

int main(int argc, char ** argv)
{
    /** ROS initialization **/
    ros::init(argc, argv, "rsi_client");
    ros::NodeHandle nh;
    ros::Subscriber positionCommandSubscriber = nh.subscribe("/kuka_arm/arm_controller/position_command", 1000, jointPositionCallback);

    /** Setup connection **/
    io_service service;
    CommandClient client("127.0.0.1", 2002, service);
    client.connect();

    /** Settings **/
    /// Accelerations
    double a[DOF] = {5, 5, 5, 5, 5, 5};
    JointVal accel(a);

    /// Velocities
    double v[DOF] = {7, 7, 7, 7, 7, 7};
    JointVal vel(v);

    std::stringstream ss("");
    std::string cmd;

    int dt = TIME_STEP*1000;
    int maxNum = 0;

    // std::cout << "Time Step: " << TIME_STEP << " s  and " << TIME_STEP*1000  << " and " << dt << " ms" << std::endl;

    // time step must be the same
    TrajectoryGenerator trajGen(accel, vel, TIME_STEP);
    ros::Rate r(100);

    while (ros::ok()) {
        // for (size_t i = 0; i < 6; ++i) {
        //     std::cout << "A" << (i + 1) << ": "; std::cin >> desiredJntAng(i);
        // }
        // std::cout << std::endl;
        ros::spinOnce();

        if (receive) {
            if (!trajGen.generateTrajectory(currJntAng, desiredJntAng)) return 1;

            for (size_t i = 0; i < trajGen.time.size(); ++i) {

                ang = trajGen.qTraj[i];
                ss << "<AKorr A1=\"" << ang(0) << "\" A2=\"" << ang(1) << "\" A3=\"" << ang(2) << "\" A4=\"" << ang(3) << "\" A5=\"" << ang(4) << "\" A6=\"" << ang(5) << "\" />" << std::endl;
                cmd = ss.str();
                client.send(cmd);
                cmd = client.read();
                // std::cout << cmd.data() << std::endl;
                // std::cout << i << "   -----------------------------------------------\n" << std::endl;

                // clear
                ss.str("");
                boost::this_thread::sleep_for(boost::chrono::milliseconds(dt));
            }
            std::cout << "Successful!" << std::endl;
            // displayJV(currJntAng,    "[q_i] \t");
            // displayJV(desiredJntAng, "[q_e] \t");
            currJntAng = desiredJntAng;
            receive = false;
        }
        r.sleep();
    }

    return 0;
}