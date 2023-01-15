/**
 * @file mat_mul_cpu.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "benchmark/mat_mul_cpu.hpp"

float* mat_mul_cpu(float* dst, const float* src1, const float* src2, const std::size_t N) 
{
    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t j = 0; j < N; j++) {
            float sum = 0;
            for (std::size_t k = 0; k < N; k++)
                 sum += src1[i * N + k] * src2[k * N + j];
            dst[i * N + j] = sum;
        }
    }
    return dst;
}