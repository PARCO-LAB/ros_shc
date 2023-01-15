/**
 * @file mat_mul_cpu.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef ROS_SHC_BENCHMARK_MAT_MUL_CPU_HPP
#define ROS_SHC_BENCHMARK_MAT_MUL_CPU_HPP

#include <cstddef>

float* mat_mul_cpu(float* dst, const float* src1, const float* src2, const std::size_t N);

#endif // ROS_SHC_BENCHMARK_MAT_MUL_CPU_HPP