#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_shc/msg/input_data.hpp"
#include "ros_shc/msg/output_data.hpp"

#include "benchmark/benchmark.hpp"
#include "benchmark/mat_mul_cpu.hpp"
#include "benchmark/mat_mul_gpu.cuh"
#include "parco/analysis.hpp"


using namespace std::chrono_literals;


class ZcProducer : public rclcpp::Node
{
public:
    ZcProducer(std::string id, std::size_t length, std::size_t num_iter = 1)
        /* ===================== EDIT HERE ================================== */
        // Node option for intra process: 
        // : Node("node_name", rclcpp::NodeOptions().use_intra_process_comms(true))
        : Node("zc_producer_" + id /*, ... */)
        /* ================================================================== */
        , _count(0)
        , _length(length)
        , _num_iter(num_iter)
    {
        _publisher = this->create_publisher<ros_shc::msg::InputData>(
            "input" + id, 
            rclcpp::QoS(1).reliable());
        _timer = this->create_wall_timer(
            500ms, 
            std::bind(&ZcProducer::timer_callback, this));
    }

private:
    void timer_callback()
    {
        /* ===================== EDIT HERE ================================== */
        // Standard copy version: 
        auto message = ros_shc::msg::InputData();
        // Zero copy version: 
        // auto msg_ptr = std::make_unique<...>();
        // auto& message = *msg_ptr;
        /* ================================================================== */
        std_msgs::msg::MultiArrayDimension dim;
        dim.size = _length;
        message.mat1.layout.dim = {dim};
        init<float>(message.mat1.data, _length * _length);
        message.mat2.layout.dim = {dim};
        init<float>(message.mat2.data, _length * _length);

        RCLCPP_INFO(
            this->get_logger(), 
            "Publishing: { N: %zu, frame_id: %zu }", _length, _count);
        message.header.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        message.header.frame_id = _count;
        /* ===================== EDIT HERE ================================== */
        // Standard copy version: 
        _publisher->publish(message);
        // Zero copy version: 
        // _publisher->publish(std::move(<message-pointer>));
        /* ================================================================== */

        if (_count > _num_iter) rclcpp::shutdown();
        _count++;
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<ros_shc::msg::InputData>::SharedPtr _publisher;
    std::size_t _count;
    std::size_t _length;
    std::size_t _num_iter;
};


class ZcMatMulGpu : public rclcpp::Node
{
public:
    ZcMatMulGpu(std::string id)
        /* ===================== EDIT HERE ================================== */
        : Node("zc_mat_mul_gpu_" + id /*, ... */)
        /* ================================================================== */
        , _latency()
        , _exec()
    {
        _publisher = this->create_publisher<ros_shc::msg::OutputData>(
            "gpu_output" + id, 
            rclcpp::QoS(1).reliable());

        _subscription = this->create_subscription<ros_shc::msg::InputData>(
            "input" + id, rclcpp::QoS(1).reliable(), 
            std::bind(&ZcMatMulGpu::callback, this, std::placeholders::_1));
    }

    ~ZcMatMulGpu() 
    {
        parco::analysis::Matrix<double> matrix_analysis(
            {{"latency", _latency.values()},
             {"gpu_exec", _exec.values()}}, 
            get_name());
        matrix_analysis.show_analysis();
    }

private:
    /* ===================== EDIT HERE ================================== */
    void callback(const ros_shc::msg::InputData::SharedPtr in_msg)
    /* ================================================================== */
    {
        std::size_t N = in_msg->mat1.layout.dim[0].size;
        auto msg_time = std::chrono::nanoseconds(in_msg->header.timestamp);
        auto t = std::chrono::high_resolution_clock::now().time_since_epoch();
        std::chrono::duration<double, std::milli> elapsed = t - msg_time;
        RCLCPP_INFO(
            this->get_logger(), 
            "Received: { N: %zu, mat1_size: %zu, mat2_size: %zu, frame_id: %zu, latency: %f }", 
            N, in_msg->mat1.data.size(), in_msg->mat2.data.size(), 
            in_msg->header.frame_id, elapsed.count());
        _latency.values().push_back(elapsed.count());

        _exec.start();
        /* ===================== EDIT HERE ================================== */
        auto out_msg = ros_shc::msg::OutputData();
        /* ================================================================== */
        out_msg.mat.data.resize(N * N);
        mat_mul_gpu(out_msg.mat.data.data(), 
                    in_msg->mat1.data.data(),
                    in_msg->mat2.data.data(), N);
        _exec.stop();

        out_msg.header.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        out_msg.header.frame_id = in_msg->header.frame_id;
        std_msgs::msg::MultiArrayDimension dim;
        dim.size = N;
        out_msg.mat.layout.dim = {dim};
        /* ===================== EDIT HERE ================================== */
        _publisher->publish(out_msg);
        /* ================================================================== */
    }

    rclcpp::Publisher<ros_shc::msg::OutputData>::SharedPtr _publisher;
    rclcpp::Subscription<ros_shc::msg::InputData>::SharedPtr _subscription;
    parco::analysis::TimeVector<double> _latency;
    parco::analysis::TimeVector<double> _exec;
};


class ZcMatMulCpu : public rclcpp::Node
{
public:
    ZcMatMulCpu(std::string id)
        /* ===================== EDIT HERE ================================== */
        : Node("zc_mat_mul_cpu_" + id /*, ... */)
        /* ================================================================== */
        , _latency()
        , _exec()
    {
        _publisher = this->create_publisher<ros_shc::msg::OutputData>(
            "cpu_output" + id, 
            rclcpp::QoS(1).reliable());

        _subscription = this->create_subscription<ros_shc::msg::InputData>(
            "input" + id, rclcpp::QoS(1).reliable(), 
            std::bind(&ZcMatMulCpu::callback, this, std::placeholders::_1));
    }

    ~ZcMatMulCpu() 
    {
        parco::analysis::Matrix<double> matrix_analysis(
            {{"latency", _latency.values()},
             {"cpu_exec", _exec.values()}}, 
            get_name());
        matrix_analysis.show_analysis();
    }

private:
    /* ===================== EDIT HERE ================================== */
    // Standard copy version: 
    void callback(const ros_shc::msg::InputData::SharedPtr in_msg)
    // Zero copy: 
    // void callback(const namespace::type::UniquePtr in_msg)
    /* ================================================================== */
    {
        std::size_t N = in_msg->mat1.layout.dim[0].size;
        auto msg_time = std::chrono::nanoseconds(in_msg->header.timestamp);
        auto t = std::chrono::high_resolution_clock::now().time_since_epoch();
        std::chrono::duration<double, std::milli> elapsed = t - msg_time;
        RCLCPP_INFO(
            this->get_logger(), 
            "Received: { N: %zu, mat1_size: %zu, mat2_size: %zu, frame_id: %zu, latency: %f }", 
            N, in_msg->mat1.data.size(), in_msg->mat2.data.size(), 
            in_msg->header.frame_id, elapsed.count());
        _latency.values().push_back(elapsed.count());

        _exec.start();
        /* ===================== EDIT HERE ================================== */
        auto out_msg = ros_shc::msg::OutputData();
        /* ================================================================== */
        out_msg.mat.data.resize(N * N);
        mat_mul_cpu(out_msg.mat.data.data(), 
                    in_msg->mat1.data.data(),
                    in_msg->mat2.data.data(), N);
        _exec.stop();

        out_msg.header.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        out_msg.header.frame_id = in_msg->header.frame_id;
        std_msgs::msg::MultiArrayDimension dim;
        dim.size = N;
        out_msg.mat.layout.dim = {dim};
        /* ===================== EDIT HERE ================================== */
        _publisher->publish(out_msg);
        /* ================================================================== */
    }

    rclcpp::Publisher<ros_shc::msg::OutputData>::SharedPtr _publisher;
    rclcpp::Subscription<ros_shc::msg::InputData>::SharedPtr _subscription;
    parco::analysis::TimeVector<double> _latency;
    parco::analysis::TimeVector<double> _exec;
};


class ZcChecker : public rclcpp::Node
{
public:
    ZcChecker(std::string id)
        /* ===================== EDIT HERE ================================== */
        : Node("zc_checker_" + id /*, ... */)
        /* ================================================================== */
        , _cpu_latency()
        , _gpu_latency()
        , _cpu_frame_id(-1)
        , _gpu_frame_id(-1)
        , _cpu_mat()
        , _gpu_mat()
    {
        _cpu_subscription = this->create_subscription<ros_shc::msg::OutputData>(
            "cpu_output" + id, rclcpp::QoS(1).reliable(), 
            std::bind(&ZcChecker::cpu_callback, this, std::placeholders::_1));
        _gpu_subscription = this->create_subscription<ros_shc::msg::OutputData>(
            "gpu_output" + id, rclcpp::QoS(1).reliable(), 
            std::bind(&ZcChecker::gpu_callback, this, std::placeholders::_1));
    }

    ~ZcChecker() 
    {
        parco::analysis::Matrix<double> matrix_analysis(
            {{"cpu_latency", _cpu_latency.values()},
             {"gpu_latency", _gpu_latency.values()}}, 
            get_name());
        matrix_analysis.show_analysis();
    }

private:
    /* ===================== EDIT HERE ================================== */
    void cpu_callback(const ros_shc::msg::OutputData::SharedPtr in_msg)
    /* ================================================================== */
    {
        std::size_t N = in_msg->mat.layout.dim[0].size;
        auto msg_time = std::chrono::nanoseconds(in_msg->header.timestamp);
        auto t = std::chrono::high_resolution_clock::now().time_since_epoch();
        std::chrono::duration<double, std::milli> elapsed = t - msg_time;
        RCLCPP_INFO(
            this->get_logger(), 
            "Received from CPU: { N: %zu, mat_size: %zu, frame_id: %zu, latency: %f }", 
            N, in_msg->mat.data.size(), in_msg->header.frame_id, elapsed.count());
        _cpu_latency.values().push_back(elapsed.count());

        if (_cpu_frame_id == -1 || _gpu_frame_id > _cpu_frame_id)
        {
            _cpu_frame_id = in_msg->header.frame_id;
            _cpu_mat = in_msg->mat.data;
        }
        _check();
    }

    /* ===================== EDIT HERE ================================== */
    void gpu_callback(const ros_shc::msg::OutputData::SharedPtr in_msg)
    /* ================================================================== */
    {
        std::size_t N = in_msg->mat.layout.dim[0].size;
        auto msg_time = std::chrono::nanoseconds(in_msg->header.timestamp);
        auto t = std::chrono::high_resolution_clock::now().time_since_epoch();
        std::chrono::duration<double, std::milli> elapsed = t - msg_time;
        RCLCPP_INFO(
            this->get_logger(), 
            "Received from GPU: { N: %zu, mat_size: %zu, frame_id: %zu, latency: %f }", 
            N, in_msg->mat.data.size(), in_msg->header.frame_id, elapsed.count());
        _gpu_latency.values().push_back(elapsed.count());

        if (_gpu_frame_id == -1 || _cpu_frame_id > _gpu_frame_id)
        {
            _gpu_frame_id = in_msg->header.frame_id;
            _gpu_mat = in_msg->mat.data;
        }
        _check();
    }

    void _check() 
    {
        if (_cpu_frame_id == _gpu_frame_id)
        {
            if (check<float>(_cpu_mat.data(), _gpu_mat.data(), _gpu_mat.size()))
            {
                RCLCPP_INFO(
                    this->get_logger(), 
                    "Check OK on frame_id: %zu", _cpu_frame_id);
            }
            else 
            {
                RCLCPP_INFO(
                    this->get_logger(), 
                    "Check ERROR on frame_id: %zu", _cpu_frame_id);
            }
            _cpu_frame_id = -1;
            _gpu_frame_id = -1;
        }
    }

    rclcpp::Subscription<ros_shc::msg::OutputData>::SharedPtr _cpu_subscription;
    rclcpp::Subscription<ros_shc::msg::OutputData>::SharedPtr _gpu_subscription;
    parco::analysis::TimeVector<double> _cpu_latency;
    parco::analysis::TimeVector<double> _gpu_latency;

    std::int64_t _cpu_frame_id;
    std::int64_t _gpu_frame_id;
    std::vector<float> _cpu_mat, _gpu_mat;
};