#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <iostream>

#include "kuka_rsi3/CommandComunication.hpp"
#include "kuka_rsi3/TrajectoryGenerator.hpp"

#define TIME_STEP 0.004

int main(int argc, char ** argv)
{
    /** Setup connection **/
    io_service service;
    CommandClient client("127.0.0.1", 2002, service);
    client.connect();

    /** Settings **/
    /// Accelerations
    double a[DOF] = {5, 5, 5, 5, 5, 5};
    JointVal accel(a);

    /// Velocities
    double v[DOF] = {10, 10, 10, 10, 10, 10}
    JointVal vel(v);

    JointVal currJntAng, desiredJntAng, ang;

    std::cout << "Insert value for each axis (A1, A2, ... , A6) in degree and press ENTER." << std::endl;
    std::cout << "For example: " << std::endl;
    std::cout << "A1: 60.1"  << std::endl;
    std::cout << "A2: 0.1"       << std::endl;
    std::cout << "A3: 0"         << std::endl;
    std::cout << "A4: 90"        << std::endl;
    std::cout << "A5: -30"       << std::endl;
    std::cout << "A6: 60"       << std::endl;
    std::cout << "--------------------------------------------------\n" << std::endl;

    std::stringstream ss("");
    std::string cmd;

    int dt = TIME_STEP*1000;
    int maxNum = 0;

    // std::cout << "Time Step: " << TIME_STEP << " s  and " << TIME_STEP*1000  << " and " << dt << " ms" << std::endl;

    // time step must be the same
    TrajectoryGenerator trajGen(accel, vel, TIME_STEP);

    while (true) {
        for (size_t i = 0; i < 6; ++i) {
            std::cout << "A" << (i + 1) << ": "; std::cin >> desiredJntAng(i);
        }
        std::cout << std::endl;

        if (!trajGen.generateTrajectory(currJntAng, desiredJntAng)) return 1;

        for (size_t i = 0; i < trajGen.time.size(); ++i) {

            ang = trajGen.qTraj[i];
            ss << "<AKorr A1=\"" << ang(0) << "\" A2=\"" << ang(1) << "\" A3=\"" << ang(2) << "\" A4=\"" << ang(3) << "\" A5=\"" << ang(4) << "\" A6=\"" << ang(5) << "\" />" << std::endl;
            cmd = ss.str();
            client.send(cmd);
            cmd = client.read();
            std::cout << cmd.data() << std::endl;
            std::cout << i << "   -----------------------------------------------\n" << std::endl;

            // clear
            ss.str("");
            boost::this_thread::sleep_for(boost::chrono::milliseconds(dt));
        }
        currJntAng = desiredJntAng;
    }

    return 0;
}
