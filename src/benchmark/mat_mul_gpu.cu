/**
 * @file mat_mul_gpu.cu
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "benchmark/mat_mul_gpu.cuh"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include "parco/Timer.cuh"
#include "parco/CheckError.cuh"


const int BLOCK_SIZE_X = 16;
const int BLOCK_SIZE_Y = 16;

__global__
void matrixMultiplicationKernel(const float* d_matrixA,
                                const float* d_matrixB,
                                std::size_t        N,
                                float*       d_matrixC) {
    // Calculate the row index of the Pd element and M
    std::size_t Row = blockIdx.y*blockDim.y + threadIdx.y;
    // Calculate the column index of Pd and N
    std::size_t Col = blockIdx.x*blockDim.x + threadIdx.x;
    
    float Pvalue = 0;
    // each thread computes one element of the block sub-matrix
    for (std::size_t k = 0; k < N; ++k)
        Pvalue += d_matrixA[Row*N+k] * d_matrixB[Col+k*N];
    
    d_matrixC[Row*N+Col] = Pvalue;
}

CudaMem mat_mul_gpu_init(const std::size_t N)
{
    CudaMem info;
    SAFE_CALL( cudaMalloc( &info.d_matrixA, N*N * sizeof(float) ));
    SAFE_CALL( cudaMalloc( &info.d_matrixB, N*N * sizeof(float) ));
    SAFE_CALL( cudaMalloc( &info.d_matrixC, N*N * sizeof(float) ));
    return info;
}


void mat_mul_gpu_deinit(CudaMem info)
{
    SAFE_CALL( cudaFree( info.d_matrixA ) );
    SAFE_CALL( cudaFree( info.d_matrixB ) );
    SAFE_CALL( cudaFree( info.d_matrixC ) );
}


float* mat_mul_gpu(float* dst, const float* src1, const float* src2, const std::size_t N)
{
    float *d_matrixA, *d_matrixB, *d_matrixC;
    SAFE_CALL( cudaMalloc( &d_matrixA, N*N * sizeof(float) ));
    SAFE_CALL( cudaMalloc( &d_matrixB, N*N * sizeof(float) ));
    SAFE_CALL( cudaMalloc( &d_matrixC, N*N * sizeof(float) ));

    // -------------------------------------------------------------------------
    // COPY DATA FROM HOST TO DEVIE
    SAFE_CALL( cudaMemcpy( d_matrixA, src1, N*N * sizeof(float), cudaMemcpyHostToDevice));
    SAFE_CALL( cudaMemcpy( d_matrixB, src2, N*N * sizeof(float), cudaMemcpyHostToDevice));

    // -------------------------------------------------------------------------
    // DEVICE INIT
    dim3 DimGrid(N/BLOCK_SIZE_X, N/BLOCK_SIZE_Y, 1);
    if (N%BLOCK_SIZE_X) DimGrid.x++;
    if (N%BLOCK_SIZE_Y) DimGrid.y++;
    dim3 DimBlock(BLOCK_SIZE_X, BLOCK_SIZE_Y, 1);
    
    // -------------------------------------------------------------------------
    // DEVICE EXECUTION
    matrixMultiplicationKernel<<< DimGrid,DimBlock>>> (d_matrixA, d_matrixB, N, d_matrixC);
    CHECK_CUDA_ERROR

    // -------------------------------------------------------------------------
    // COPY DATA FROM DEVICE TO HOST
    SAFE_CALL( cudaMemcpy( dst, d_matrixC, N*N * sizeof(float), cudaMemcpyDeviceToHost));

    SAFE_CALL( cudaFree( d_matrixA ) );
    SAFE_CALL( cudaFree( d_matrixB ) );
    SAFE_CALL( cudaFree( d_matrixC ) );

    return dst;
}


float* mat_mul_gpu_exec(float* dst, const float* src1, const float* src2, const std::size_t N, CudaMem info) 
{
    // -------------------------------------------------------------------------
    // COPY DATA FROM HOST TO DEVIE
    SAFE_CALL( cudaMemcpy( info.d_matrixA, src1, N*N * sizeof(float), cudaMemcpyHostToDevice));
    SAFE_CALL( cudaMemcpy( info.d_matrixB, src2, N*N * sizeof(float), cudaMemcpyHostToDevice));

    // -------------------------------------------------------------------------
    // DEVICE INIT
    dim3 DimGrid(N/BLOCK_SIZE_X, N/BLOCK_SIZE_Y, 1);
    if (N%BLOCK_SIZE_X) DimGrid.x++;
    if (N%BLOCK_SIZE_Y) DimGrid.y++;
    dim3 DimBlock(BLOCK_SIZE_X, BLOCK_SIZE_Y, 1);
    
    // -------------------------------------------------------------------------
    // DEVICE EXECUTION
    matrixMultiplicationKernel<<< DimGrid,DimBlock>>> (info.d_matrixA, info.d_matrixB, N, info.d_matrixC);
    CHECK_CUDA_ERROR

    // -------------------------------------------------------------------------
    // COPY DATA FROM DEVICE TO HOST
    SAFE_CALL( cudaMemcpy( dst, info.d_matrixC, N*N * sizeof(float), cudaMemcpyDeviceToHost));

    return dst;
}