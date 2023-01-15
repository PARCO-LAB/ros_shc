#include "sc_nodes.hpp"

#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 3)
    {
        std::cout << "User arguments error: need of 2 params" << std::endl;
        return -1;
    }
    char *id = argv[1];
    std::size_t input_length = atoi(argv[2]);
    const std::size_t iterations = 100;
    rclcpp::spin(std::make_shared<ScProducer>(id, input_length, iterations));
    rclcpp::shutdown();
    return 0;
}