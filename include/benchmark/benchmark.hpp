/**
 * @file benchmark.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SHC_BENCHMARK_BENCHMARK_HPP
#define SHC_BENCHMARK_BENCHMARK_HPP

#include <vector>

template<typename T>
void init(std::vector<T>& v, std::size_t length)
{
    v.resize(length);
    for (std::size_t i = 0; i < length; ++i)
    {
        v[i] = i % 10;
    }
}

template<typename T>
bool check(const T* v1, const T* v2, std::size_t length)
{
    for (std::size_t i = 0; i < length; ++i)
    {
        if (v1[i] - v2[i] >= 0.00001) {
            std::cout << "[" << i << "] " <<  v1[i] << " != " << v2[i] << std::endl;
            return false;
        }
    }
    return true;
}

#endif // SHC_BENCHMARK_BENCHMARK_HPP