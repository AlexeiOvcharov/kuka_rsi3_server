#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <iostream>

#include "kuka_rsi3/CommandComunication.hpp"
#include "kuka_rsi3/TrajectoryGenerator.hpp"

#define TIME_STEP 0.01
#define ACCEL 3
#define VEL 5

int main(int argc, char ** argv)
{
    io_service service;
    CommandClient client("127.0.0.1", 2002, service);
    client.connect();
    double A[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double currA[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::cout << "Insert value for each axis (A1, A2, ... , A6) in degree and press ENTER." << std::endl;
    std::cout << "For example: " << std::endl;
    // std::cout << "A1: 60.1"  << std::endl;
    // std::cout << "A2: 0.1"       << std::endl;
    // std::cout << "A3: 0"         << std::endl;
    // std::cout << "A4: 90"        << std::endl;
    // std::cout << "A5: -30"       << std::endl;
    std::cout << "A6: 60"       << std::endl;
    std::cout << "--------------------------------------------------\n" << std::endl;

    std::stringstream ss("");
    std::string cmd;

    int dt = (int)TIME_STEP*1000;
    int maxNum = 0;

    // time step must be the same
    TrajectoryGenerator trajGen(ACCEL, VEL, TIME_STEP);

    while (true) {
        for (size_t i = 5; i < 6; ++i) {
            std::cout << "A" << (i + 1) << ": "; std::cin >> A[i];
        }
        std::cout << std::endl;

        if (!trajGen.generateTrajectory(currA[0], A[0])) return 1;

        for (size_t i = 0; i < trajGen.time.size(); ++i) {

            ss << "<AKorr A1=\"" << A[0] << "\" A2=\"" << A[1] << "\" A3=\"" << A[2] << "\" A4=\"" << A[3] << "\" A5=\"" << A[4] << "\" A6=\"" << trajGen.qTraj[i] << "\" />" << std::endl;
            cmd = ss.str();
            client.send(cmd);
            cmd = client.read();
            std::cout << cmd.data() << std::endl;
            std::cout << i << "   -----------------------------------------------\n" << std::endl;

            // clear
            ss.str("");
            boost::this_thread::sleep_for(boost::chrono::milliseconds(dt));
        }
        currA[5] = A[5];
    }

    return 0;
}
