#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem/fstream.hpp>

#include <yaml-cpp/yaml.h>  // For configuration of project
#include <pugixml.hpp>      // Work with xml tree
#include <string>
#include <iostream>         // std::cout
#include <sstream>          // std::stringstream
#include <cstdlib>

// ROS
#include <ros/package.h>
#include <ros/ros.h>

#include "kuka_rsi3/CommandComunication.hpp"
#include <sensor_msgs/JointState.h>

#define OFFSET_A1 0
#define OFFSET_A2 -90
#define OFFSET_A3 90
#define OFFSET_A4 0
#define OFFSET_A5 0
#define OFFSET_A6 0

using namespace boost::asio;

boost::filesystem::path p{"test.dat"};
boost::filesystem::ofstream ofs{p};

double A[6] = {0, 0, 0, 0, 0, 0};
std::vector<double> JV(6);
// pugi::xml_node AK;

void receiveCommand()
{
    // TODO better catch of errors
    pugi::xml_document commandXML;
    std::string cmd;
    std::string success = "Seccessfull!";
    io_service serv;
    CommandServer server("127.0.0.1", 2002, serv);
    while (true && !server.is_ok()) {
        server.accept();

        while (server.is_ok()) {
            cmd = server.read();
            commandXML.load_string(cmd.c_str());

            A[0] = commandXML.child("AKorr").attribute("A1").as_double();
            A[1] = commandXML.child("AKorr").attribute("A2").as_double();
            A[2] = commandXML.child("AKorr").attribute("A3").as_double();
            A[3] = commandXML.child("AKorr").attribute("A4").as_double();
            A[4] = commandXML.child("AKorr").attribute("A5").as_double();
            A[5] = commandXML.child("AKorr").attribute("A6").as_double();
            // ROS_INFO_STREAM("Info: " << A[0] << ", " << A[1] << ", " << A[2] << ", " << A[3] << ", " << A[4] << ", " << A[5] << " --");
            // AK = commandXML.child("AKorr");

            server.send(success);
        }
        std::cout << "[TCP] Connection lost." << std::endl;
    }
}

void publishState(sensor_msgs::JointState stateMSG, size_t rate)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher statePublisher = node->advertise<sensor_msgs::JointState> ("/joint_states", 10);
    ros::Duration(1).sleep();

    ROS_INFO("Start RSI state publisher.");
    ros::Rate r(rate);

    while(ros::ok()) {

        // Fill state message
        stateMSG.header.stamp = ros::Time::now();
        stateMSG.position = JV;
        statePublisher.publish(stateMSG);

        r.sleep();
    }
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rsi_server", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;

    // TODO project recofigure
    //YAML::Node config = YAML::LoadFile("../config/settings.yaml");
    //ip::address servAddres = ip::address::from_string(config["server_addres"].as<std::string>());
    //int servPort = config["server_port"].as<int>();

    /*** Setup configuration ***/
    std::string packagePath = ros::package::getPath("kuka_rsi3_server");

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(std::string(packagePath + "/RSI/Painter_RSI.xml").c_str());

    pugi::xml_node config =  doc.child("ROOT").child("CONFIG");
    const ip::address servAddres = ip::address::from_string(config.child_value("IP_NUMBER"));
    const int servPort = std::stoi(config.child_value("PORT"));
    const char* senType = config.child_value("SENTYPE");
    const char* description = "Painter";

    /*** Createing response message ***/
    pugi::xml_document resp;
    result = resp.load_file(std::string(packagePath + "/RSI/Send_MSG.xml").c_str());

    resp.child("Sen").attribute("Type").set_value(senType);
    resp.child("Sen").child("EStr").last_child().set_value(description);
    pugi::xml_node ipoc = resp.child("Sen").child("IPOC").last_child();
    pugi::xml_node AK = resp.child("Sen").child("AKorr");

    ipoc.set_value("53");

    std::stringstream ss;
    resp.save(ss, "\t", pugi::format_raw | pugi::format_no_declaration);
    std::string respMsg = ss.str();
    // std::cout << respMsg << std::endl;
    ss.str("");

    // Create and fill state message
    sensor_msgs::JointState stateMSG;
    stateMSG.name = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};
    stateMSG.effort.resize(6);
    stateMSG.velocity.resize(6);
    stateMSG.position.resize(6);
    size_t stateFreq = 50;


    // Define socket service and other
    io_service service;
    boost::array<char, 1024> buff;
    std::size_t receivedBytes = 0;

    std::cout << "[UDP] Create socket at addr: (" << servAddres << ", " << servPort << ")" << std::endl;
    ip::udp::socket socket(service, ip::udp::endpoint(servAddres, servPort));

    // Simple test of tcp client
    boost::thread cmdThread{receiveCommand};
    boost::thread stateThread(publishState, stateMSG, stateFreq);

    pugi::xml_document receiveXML; // Declare receive xml tree

    try {

        ip::udp::endpoint remote_endpoint;
        boost::system::error_code error;
        boost::system::error_code ignored_error;
        bool receiveStart = false;

        ros::Duration(5).sleep();
        std::cout << GREEN << "Ready to work!" << RESET << std::endl;
        while (ros::ok()) {

            receivedBytes = socket.receive_from(buffer(buff), remote_endpoint, 0, error); // TODO Check the cleanless of buff
            if (!receiveStart) {
                receiveStart = true;
                std::cout << "[UDP] Start working ..." << std::endl;
            }

            /// Work with receiving message
            receiveXML.load_string(buff.c_array());
            JV[0] = (receiveXML.child("Rob").child("AIPos").attribute("A1").as_double()) * M_PI/180;
            JV[1] = (receiveXML.child("Rob").child("AIPos").attribute("A2").as_double()) * M_PI/180;
            JV[2] = (receiveXML.child("Rob").child("AIPos").attribute("A3").as_double()) * M_PI/180;
            JV[3] = (receiveXML.child("Rob").child("AIPos").attribute("A4").as_double()) * M_PI/180;
            JV[4] = (receiveXML.child("Rob").child("AIPos").attribute("A5").as_double()) * M_PI/180;
            JV[5] = (receiveXML.child("Rob").child("AIPos").attribute("A6").as_double()) * M_PI/180;

            // std::cout << "*** Receive ------------------------------------------ " << receivedBytes << "bytes --- \n" << buff.c_array() << std::endl;
            receiveXML.save(ss, "\t", pugi::format_raw | pugi::format_no_declaration);
            respMsg = ss.str();
            std::cout << respMsg << std::endl;
            ss.str("");

            ipoc.set_value(receiveXML.child("Rob").child_value("IPOC"));
            AK.attribute("A1").set_value(A[0]);
            AK.attribute("A2").set_value(A[1]);
            AK.attribute("A3").set_value(A[2]);
            AK.attribute("A4").set_value(A[3]);
            AK.attribute("A5").set_value(A[4]);
            AK.attribute("A6").set_value(A[5]);
            // ROS_INFO_STREAM("Info: " << A[0] << ", " << A[1] << ", " << A[2] << ", " << A[3] << ", " << A[4] << ", " << A[5] << " --");
            // resp.child("Sen").child("AKorr") = AK;
            resp.save(ss, "\t", pugi::format_raw | pugi::format_no_declaration);
            respMsg = ss.str();
            // std::cout << respMsg << std::endl;
            if (error && error != error::message_size)
                throw boost::system::system_error(error);

            // Simple trajectory regulator
            socket.send_to(buffer(respMsg), remote_endpoint, 0, ignored_error);

            /// Clear
            ss.str("");

            /// Threads communication
            cmdThread.interrupt();
            stateThread.interrupt();
        }
        cmdThread.join();
        stateThread.join();
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        cmdThread.join();
        service.stop();
        service.~io_service();
    }

    return 0;
}