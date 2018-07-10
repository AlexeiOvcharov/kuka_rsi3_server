#ifndef PRETTY_PRINT
#define PRETTY_PRINT

#include <bprinter/table_printer.h>
#include <kuka_rsi3/DataTypes.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define _USE_MATH_DEFINES
#define USE_BOOST_KARMA
#include <math.h>

#if defined(USE_BOOST_KARMA)
#include <boost/spirit/include/karma.hpp>
namespace karma = boost::spirit::karma;
#endif

using bprinter::TablePrinter;

class PrettyPrint
{
    public:
        PrettyPrint() : trajInfoTable(&std::cout)
        {
            trajInfoTable.AddColumn("Vel",      10);
            trajInfoTable.AddColumn("Accel",    10);
            trajInfoTable.AddColumn("q_i",      20);
            trajInfoTable.AddColumn("q_e",      20);
        }
        ~PrettyPrint()
        {}

        void printTrajInfo(const TrajectoryInformation & info) {

            trajInfoTable.PrintHeader();
            for (size_t i = 0; i < DOF; ++i)
                trajInfoTable << info.vel(i) << info.accel(i) << info.q_i(i) << info.q_e(i);
            trajInfoTable.PrintFooter();
        }

    private:
        TablePrinter trajInfoTable;
};

#endif