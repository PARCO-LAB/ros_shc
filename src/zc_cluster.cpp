#include "zc_nodes.hpp"

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

    auto producer = std::make_shared<ZcProducer>(id, input_length, iterations);
    auto mat_mul_cpu = std::make_shared<ZcMatMulCpu>(id);
    auto mat_mul_gpu = std::make_shared<ZcMatMulGpu>(id);
    auto checker = std::make_shared<ZcChecker>(id);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(producer);
    executor.add_node(mat_mul_cpu);
    executor.add_node(mat_mul_gpu);
    executor.add_node(checker);
    
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}