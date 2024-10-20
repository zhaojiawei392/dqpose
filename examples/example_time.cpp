/** 
 *     This file is part of dqpose.
 *  
 *     dqpose is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dqpose is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dqpose. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file examples/example_quat.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 */

#include "dqpose.hpp"
#include <vector>
#include <chrono>


int main() {
using namespace dqpose;
    std::vector<Quatf> q_vec;
    
    auto start0 = std::chrono::high_resolution_clock::now();

    q_vec.emplace_back(Quatf(0,0,0,0));
    for (size_t i=1; i<10000000; i++){
        q_vec.emplace_back(Quatf(i,i,i,i) * q_vec[i-1] * Rotf(Quatf(i,i,i,i)));
    }
    auto end0 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed0 = end0 - start0;

    std::cout << "reference Elapsed time: " << elapsed0.count() <<" ms\n";

}

