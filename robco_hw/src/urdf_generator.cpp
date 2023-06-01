#include <cstdlib>
#include <getopt.h>
#include <robcomm/robcomm.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>
#include <iostream>

robcomm::Robot robot;

const int INIT_TIMEOUT = 10;

void printHelp() {
    std::cerr << "Usage: urdf_generator --ip IP --rx-port RX_PORT --tx-port TX_PORT" << std::endl;
    std::cerr << "Generates URDF for robot connected to IP, RX_PORT, TX_PORT and prints it to stdout." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Connection parameters:" << std::endl;
    std::cerr << "  -i, --ip: IP address of the robot controller" << std::endl;
    std::cerr << "  -r, --rx-port: local port to listen for messages from robot" << std::endl;
    std::cerr << "  -t, --tx-port: remote port to send messages for robot to" << std::endl;
}

int main(int argc, char** argv) {
    std::string outPath;
    std::string ipString;
    int localRxPort = 0;
    int remoteTxPort = 0;

    static const struct option long_options[] =
    {
        { "ip",       required_argument, 0, 'i' },
        { "rx-port",  required_argument, 0, 'r' },
        { "tx-port",  required_argument, 0, 't' },
        0
    };

    while (1) {
        int index = -1;
        struct option* opt = 0;

        int result = getopt_long(argc, argv, "i:r:t:", long_options, &index);

        if (result == -1) break;

        switch (result) {
            case 'i':
                ipString.assign(optarg);
                break;
            case 'r':
                localRxPort = strtol(optarg, nullptr, 10);
                break;
            case 't':
                remoteTxPort = strtol(optarg, nullptr, 10);
                break;
            default:
                break;
        }

    }

    robot.connect(ipString, localRxPort, remoteTxPort);

    // Wait for robot initialization
    for (int i = 0; i < 100 * INIT_TIMEOUT && !robot.is_initialized(); i ++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        robot.receive();
    }

    if (!robot.is_initialized()) {
        std::cerr << "Timeout waiting for robot to initialize" << std::endl;
        return -1;
    }

    // Generate URDF
    std::cout << "<?xml version=\"1.0\"?>" << std::endl;
    std::cout << "<robot name=\"robco\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" << std::endl;

    std::cout << "    <xacro:include filename=\"$(find robco_description)/urdf/robco_modules.xacro\"/>" << std::endl;

    for (int i = 0; i < robot.get_module_count(); i ++) {
        std::string name = "module";
        name += std::to_string(i);

        std::cout << "    <xacro:robco_module_" << std::setfill('0') << std::setw(4)
            << robot.get_module_type_id(i) << " name=\"" << name << "\""; 

        if (i < robot.get_module_count() - 1) {
            std::string nextName = "module";
            nextName += std::to_string(i+1);

            std::cout << " next=\"" << nextName << "\"";
        }

        std::cout << "/>" << std::endl;
    }

    std::cout << "</robot>" << std::endl;
}
