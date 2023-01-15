/**
 * @file mat_mul_gpu.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef ROS_SHC_BENCHMARK_MAT_MUL_GPU_HPP
#define ROS_SHC_BENCHMARK_MAT_MUL_GPU_HPP

#include <cstddef>

struct CudaMem {
    float *d_matrixA, *d_matrixB, *d_matrixC;
};

CudaMem mat_mul_gpu_init(const std::size_t N);
void mat_mul_gpu_deinit(CudaMem info);
float* mat_mul_gpu(float* dst, const float* src1, const float* src2, const std::size_t N);
float* mat_mul_gpu_exec(float* dst, const float* src1, const float* src2, const std::size_t N, CudaMem info);

#endif // ROS_SHC_BENCHMARK_MAT_MUL_GPU_HPP