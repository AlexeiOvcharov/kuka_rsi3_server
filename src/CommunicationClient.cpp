#include "kuka_rsi3/CommunicationClient.hpp"


#define OFFSET_A1 0
#define OFFSET_A2 -90
#define OFFSET_A3 90
#define OFFSET_A4 0
#define OFFSET_A5 0
#define OFFSET_A6 0
#define TIME_STEP 0.004

JointVal fromMsgToJointVal(std::vector<double> pos) {
    JointVal jv;
    jv(0) = pos[0];
    jv(1) = pos[1];
    jv(2) = pos[2];
    jv(3) = pos[3];
    jv(4) = pos[4];
    jv(5) = pos[5];
    jv(6) = pos[6];

    return jv;
}

std::vector<double> fromJointValToMsg(JointVal jv) {
    std::vector<double> pos(6);
    pos[0] = jv(0);
    pos[1] = jv(1);
    pos[2] = jv(2);
    pos[3] = jv(3);
    pos[4] = jv(4);
    pos[5] = jv(5);
    pos[6] = jv(6);

    return pos;
}

CommunicationClient::CommunicationClient(ros::NodeHandle & nh, std::string address, short int port) : node(nh)
{

    this->address = address;
    this->port = port;
    trajectoryIsActive = false;

    /// Accelerations
    double a[DOF] = {5, 5, 5, 5, 5, 5};
    accel = JointVal(a);

    /// Velocities
    double v[DOF] = {7, 7, 7, 7, 7, 7};
    vel = JointVal(v);

    dataProcessingPackagePath = ros::package::getPath("rsi_tests");
}
CommunicationClient::~CommunicationClient()
{}

void CommunicationClient::communication(std::string address, short int port)
{

    int main_dt = 1000.0/communicationFrequency;
    int waitingModeDuration = 500; // millisecons
    std::stringstream message;
    size_t trjPoint = 0;
    std::string command;
    ROS_INFO_STREAM("Execute: main_dt: " << main_dt << " ms | \t" << communicationFrequency);

    // try {

        client = new CommandClient(address, port, service);
        client->connect();

        while(true) {
            if (trajectoryIsActive) {

                // Open file
                std::stringstream csvfilePath;
                csvfilePath << dataProcessingPackagePath << "/data/data1.csv";
                ROS_INFO("File path: %s", csvfilePath.str().c_str());
                boost::filesystem::ofstream ofs{csvfilePath.str()};
                csvfilePath.str("");

                for (size_t i = 0; i < actualTrajectory.points.size(); ++i) {
                    ROS_INFO_STREAM(i << "\t" << actualTrajectory.points[i].time_from_start
                        << "\t" << actualTrajectory.points[i].positions[0]
                        << "\t" << actualTrajectory.points[i].positions[1]
                        << "\t" << actualTrajectory.points[i].positions[2]
                        << "\t" << actualTrajectory.points[i].positions[3]
                        << "\t" << actualTrajectory.points[i].positions[4]
                        << "\t" << actualTrajectory.points[i].positions[5]);

                    // Write to file
                    ofs << actualTrajectory.points[i].time_from_start;
                    for (size_t joint = 0; joint < 6; ++joint)
                        ofs << ", " << actualTrajectory.points[i].positions[joint];
                    ofs << "\n";
                }
                ofs.close();
                ROS_INFO("----------------------------------------------------------");
                if(!trajectoryLineInterpolation()) {
                    return;
                }

                // Open file
                csvfilePath << dataProcessingPackagePath << "/data/data1_interpolation.csv";
                ROS_INFO("File path: %s", csvfilePath.str().c_str());
                ofs.open(csvfilePath.str());
                csvfilePath.str("");

                for (size_t i = 0; i < actualTrajectory.points.size(); ++i) {

                    // Write to file
                    ofs << actualTrajectory.points[i].time_from_start;
                    for (size_t joint = 0; joint < 6; ++joint)
                        ofs << ", " << actualTrajectory.points[i].positions[joint];
                    ofs << "\n";

                }
            }

            while (trajectoryIsActive) {

                message.str("");
                message << "<AKorr A1=\"" << (actualTrajectory.points[trjPoint].positions[0]*180/M_PI - OFFSET_A1)
                    << "\" A2=\"" <<         (actualTrajectory.points[trjPoint].positions[1]*180/M_PI - OFFSET_A2)
                    << "\" A3=\"" <<         (actualTrajectory.points[trjPoint].positions[2]*180/M_PI - OFFSET_A3)
                    << "\" A4=\"" <<         (actualTrajectory.points[trjPoint].positions[3]*180/M_PI - OFFSET_A4)
                    << "\" A5=\"" <<         (actualTrajectory.points[trjPoint].positions[4]*180/M_PI - OFFSET_A5)
                    << "\" A6=\"" <<         (actualTrajectory.points[trjPoint].positions[5]*180/M_PI - OFFSET_A6)
                    << "\" />" << std::endl;
                command = message.str();
                // std::cout << message.str() << std::endl;
                // std::cout << "(" << trjPoint << ")" << message.str() << std::endl;

                client->send(command);

                ++trjPoint;
                if (trjPoint == actualTrajectory.points.size()) {
                    // ROS_INFO_STREAM("Successful send trajectory");
                    std::cout << "Successful" << std::endl;
                    trjPoint = 0;
                    trajectoryIsActive = false;
                }

                boost::this_thread::sleep_for(boost::chrono::milliseconds(main_dt));
            }
            boost::this_thread::sleep_for(boost::chrono::milliseconds(waitingModeDuration));
        }

    // } catch (boost::thread_interrupted&) {ROS_ERROR_STREAM("communication");}
}

void CommunicationClient::initializeArm(std::string armName)
{
    std::cout << "InitializeArm" << std::endl;
    std::stringstream topicName;
    std::stringstream actionName;

    communicationThread = new boost::thread(&CommunicationClient::communication, this, address, port);

    topicName.str("");
    topicName << armName << "/arm_controller/position_command";
    positionCommandSubscriber = node.subscribe(topicName.str(), 100, &CommunicationClient::jointPositionCallback, this);

    actionName.str("");
    actionName << armName << "/arm_controller/follow_joint_trajectory";
    trajectoryActionServer = new Server(node, actionName.str(), boost::bind(&CommunicationClient::followJointTrajectory, this, _1), false);

    trajectoryActionServer->start();
}

void CommunicationClient::jointPositionCallback(const brics_actuator::JointPositions & msg)
{

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

    TrajectoryGenerator trajGen(accel, vel, TIME_STEP);
    if (!trajGen.generateTrajectory(currJntAng, desiredJntAng)) return;

    JointVal ang;
    std::stringstream ss("");
    std::string cmd;
    int dt = 1000/communicationFrequency;
    for (size_t i = 0; i < trajGen.time.size(); ++i) {

        ang = trajGen.qTraj[i];
        ss << "<AKorr A1=\"" << ang(0) << "\" A2=\"" << ang(1) << "\" A3=\"" << ang(2) << "\" A4=\"" << ang(3) << "\" A5=\"" << ang(4) << "\" A6=\"" << ang(5) << "\" />" << std::endl;
        cmd = ss.str();
        client->send(cmd);
        cmd = client->read();
        // std::cout << cmd.data() << std::endl;
        // std::cout << i << "   -----------------------------------------------\n" << std::endl;

        // clear
        ss.str("");
        boost::this_thread::sleep_for(boost::chrono::milliseconds(dt));
    }
    std::cout << "Successful!" << std::endl;
    currJntAng = desiredJntAng;

}

void CommunicationClient::followJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal)
{
    actualTrajectory = goal->trajectory;
    ROS_INFO_STREAM("Trajectory path: ");
    // for (size_t i = 0; i < actualTrajectory.points.size(); ++i) {
    //     ROS_INFO_STREAM(i << "\t" << actualTrajectory.points[i].time_from_start.toSec() << "\t" << actualTrajectory.points[i].positions[0]);
    // }


    ROS_INFO_STREAM("Duration: " << goal->goal_time_tolerance);

    trajectoryIsActive = true;
    // communicationThread->join();

    while (trajectoryIsActive)
        ros::Duration(0.3).sleep();

    resultOfTrajExecution.error_code = 0;
    resultOfTrajExecution.error_string = "Successful";

    trajectoryActionServer->setSucceeded(resultOfTrajExecution);
}

bool CommunicationClient::trajectoryLineInterpolation()
{
    // Basic trajecotry interpolate for:
    // y = a(t - t0) + b equation

    trajectory_msgs::JointTrajectory trj;
    trajectory_msgs::JointTrajectoryPoint point;
    size_t interpPointsNum = 0;
    double main_dt = 1/communicationFrequency;
    double dt = 0, t0 = 0, t = 0;
    JointVal a, b, q2, q1, currentJV;
    q2 = fromMsgToJointVal(actualTrajectory.points[0].positions);
    ROS_INFO_STREAM("Manin freq: " << communicationFrequency);
    ROS_INFO_STREAM("Manin dt: " << main_dt);

    for (size_t inervalNum = 0; inervalNum < actualTrajectory.points.size() - 1; ++inervalNum) {

        t0 = actualTrajectory.points[inervalNum].time_from_start.toSec();
        dt = actualTrajectory.points[inervalNum + 1].time_from_start.toSec() - t0;
        interpPointsNum = dt/main_dt;
        q1 = q2;
        q2 = fromMsgToJointVal(actualTrajectory.points[inervalNum + 1].positions);

        if (interpPointsNum == 0) {
            ROS_ERROR_STREAM("Time step is less than: " << main_dt << " (" << dt << ")");
            return false;
        }

        a = (q2 - q1)/dt;
        b = q1;

        for (size_t interpPoint = 0; interpPoint < interpPointsNum; ++interpPoint) {
            currentJV = a*(t - t0) + b;
            point.positions = fromJointValToMsg(currentJV);
            point.time_from_start = ros::Duration(t);
            trj.points.push_back(point);

            t += main_dt;
        }
    }
    point.positions = actualTrajectory.points.back().positions;
    point.time_from_start = ros::Duration(t);
    trj.points.push_back(point);
    actualTrajectory = trj;
    return true;
}

void CommunicationClient::setCommunicationFrequency(double freq)
{
    communicationFrequency = freq;
}