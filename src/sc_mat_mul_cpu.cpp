#include "sc_nodes.hpp"

#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 2)
    {
        std::cout << "User arguments error: need of 1 params" << std::endl;
        return -1;
    }
    char *id = argv[1];
    rclcpp::spin(std::make_shared<ScMatMulCpu>(id));
    rclcpp::shutdown();
    return 0;
}