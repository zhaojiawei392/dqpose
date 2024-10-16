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
 *     \file examples/example_pose.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 */

#include "../include/dqpose/pose.hpp"

void constructors_demo() {
    using namespace dqpose;

    std::cout << " Constructors of --- Rotation<float> ---              \n";
    Rotf r10;                             
    Rotf r11(Unitf(0,0,1), M_PI/3);                   
    Rotf r12(Quatf(1,2,3,4));             
    Rotf r13(r12);       
    Rotf r14(Rotf{});                      
    Rotf r15(Rotd{});  
    std::cout << "Rotation<float> Constructor\n";                               
    std::cout << " - Rotf r10;                                   : " << "\n    " << r10 << "\n";
    std::cout << " - Rotf r11(Unitf(0,0,1), M_PI/3);             : " << "\n    " << r11 << "\n";
    std::cout << " - Rotf r12(Quatf(1,2,3,4));                   : " << "\n    " << r12 << "\n";
    std::cout << " - Rotf r13(r12);                              : " << "\n    " << r13 << "\n";
    std::cout << " - Rotf r14(Rotf{});                           : " << "\n    " << r14 << "\n";
    std::cout << " - Rotf r15(Rotd{});                           : " << "\n    " << r15 << "\n";


    std::cout << "\nConstructors of --- Translation<float> ---              \n";
    Tranf t10;                                  
    Tranf t11(1);                                
    Tranf t12(1,2);                              
    Tranf t13(1,2,3);               
    Tranf t14(Quatf(1,2,3,4));                                 
    Tranf t15(t14);                                    
    Tranf t16(Tranf(1,2,3));                                
    Tranf t17(Trand(4,5,6));        
    std::cout << "Scalar Constructor\n";                              
    std::cout << " - Tranf t10;                                  : " << "\n    " << t10 << "\n";
    std::cout << " - Tranf t11(1);                               : " << "\n    " << t11 << "\n";
    std::cout << " - Tranf t12(1,2);                             : " << "\n    " << t12 << "\n";
    std::cout << " - Tranf t13{1,2,3};                           : " << "\n    " << t13 << "\n";
    std::cout << " - Tranf t14(Quatf(1,2,3,4));                  : " << "\n    " << t14 << "\n";
    std::cout << " - Tranf t15(t14);                             : " << "\n    " << t15 << "\n";
    std::cout << " - Tranf t16(Tranf(1,2,3));                    : " << "\n    " << t16 << "\n";
    std::cout << " - Tranf t17(Trand(4,5,6));                    : " << "\n    " << t17 << "\n";


    std::cout << "\nConstructors of --- UnitAxis<float> ---              \n";
    Unitf u10(1,0,0);                                    
    Unitf u11(Quatf(1,2,3,4));                                    
    Unitf u12(u10);                              
    Unitf u13(Unitf(0,1,0));                                
    Unitf u14(Unitd(0,0,1));              
    std::cout << "Scalar Constructor\n";                            
    std::cout << " - Unitf u10(1,0,0);                            : " << "\n    " << u10 << "\n";
    std::cout << " - Unitf u11(Quatf(1,2,3,4));                   : " << "\n    " << u11 << "\n";
    std::cout << " - Unitf u12(u10);                              : " << "\n    " << u12 << "\n";
    std::cout << " - Unitf u13(Unitf(0,1,0));                     : " << "\n    " << u13 << "\n";
    std::cout << " - Unitf u14(Unitd(0,0,1));                     : " << "\n    " << u14 << "\n";

    std::cout << "\nConstructors of --- Pose<float> ---              \n";
    Posef p10;                                  
    Posef p11(r12, t12);                                
    Posef p12(r13);               
    Posef p13(t13);                                  
    Posef p14(p13);                                
    Posef p15(Posef{});                        
    Posef p16(Posed{});                 
    std::cout << "Scalar Constructor\n";
    std::cout << " - Posef p10();                                 : " << "\n    " << p10 << "\n";
    std::cout << " - Posef p11(r12, t12);                         : " << "\n    " << p11 << "\n";
    std::cout << " - Posef p12(r13);                              : " << "\n    " << p12 << "\n";
    std::cout << " - Posef p13(t13);                              : " << "\n    " << p13 << "\n";
    std::cout << " - Posef p14(p13);                              : " << "\n    " << p14 << "\n";
    std::cout << " - Posef p15(Posef{});                          : " << "\n    " << p15 << "\n";
    std::cout << " - Posef p16(Posed{});                          : " << "\n    " << p15 << "\n";
}

void assignments_demo() {

}

int main() {
    constructors_demo();
    assignments_demo();
}

