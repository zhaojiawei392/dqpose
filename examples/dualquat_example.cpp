/** 
 *     This file is part of dq1.
 *  
 *     dq1 is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dq1 is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dq1. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file examples/dualquat_example.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 */

#include "../src/dqpose/dualquat.hpp"

void constructors_demo() {
    using namespace dqpose;

    std::cout << " Constructors of float Dual Quaternion --- DualQuat<float> ---              \n";
    // Dual Quaternion of scalar type of float inside
    // The default Constructor, all values are uninitialized
    // Directly use a default Dual Quaternion without initialization will produce undefined behavior
    // Initialize it before use !!!
    // DualQuatf q0;                   
    // std::cout << "Default Constructor       - DualQuatf q0;                   : " << q0 << "\n";  // Undefined behavior
    // A Constructor accepts 0 to 8 scalar values
    DualQuatf q10(1);                                    
    DualQuatf q11(1,1);                                    
    DualQuatf q12(1,1,2);                              
    DualQuatf q13(1,1,2,3);                                
    DualQuatf q14(1,1,2,3,4);                                       
    DualQuatf q15(1,1,2,3,4,5);                                     
    DualQuatf q16(1,1,2,3,4,5,6);                                 
    DualQuatf q17(1,1,2,3,4,5,6,7);       
    std::cout << "Scalar Constructor\n";                               
    std::cout << " - DualQuatf q10{1};                        : " << "\n" << q10 << "\n";
    std::cout << " - DualQuatf q11{1,1};                      : " << "\n" << q11 << "\n";
    std::cout << " - DualQuatf q12{1,1,2};                    : " << "\n" << q12 << "\n";
    std::cout << " - DualQuatf q13{1,1,2,3};                  : " << "\n" << q13 << "\n";
    std::cout << " - DualQuatf q14{1,1,2,3,4};                : " << "\n" << q14 << "\n";
    std::cout << " - DualQuatf q15{1,1,2,3,4,5};              : " << "\n" << q15 << "\n";
    std::cout << " - DualQuatf q16{1,1,2,3,4,5,6};            : " << "\n" << q16 << "\n";
    std::cout << " - DualQuatf q17{1,1,2,3,4,5,6,7};          : " << "\n" << q17 << "\n";
    // The Copy Constructor accepts another Dual Quaternion of ANY scalar type inside
    DualQuatf q20(q13);                                     
    DualQuatf q21(DualQuat<double>(2,2,2,2,2,2,2,2));              
    std::cout << "Copy Constructor\n";              
    std::cout << " - DualQuatf q20(q13);                     : " << "\n" << q20 << "\n";
    std::cout << " - DualQuatf q21(DualQuat<double>(2,2,2,2,2,2,2,2));     : " << "\n" << q21 << "\n";
    // The Move Constructor ONLY accepts another r-value Dual Quaternion of SAME scalar type inside
    DualQuatf q30(DualQuat<float>(3,3,3,3,3,3,3,3));          
    std::cout << "Move Constructor\n";                                
    std::cout << " - DualQuatf q30(DualQuat<float>(3,3,3,3,3,3,3,3));            : " << "\n" << q30 << "\n";


    std::cout << "\nConstructors of float Pure Dual Quaternion --- PureDualQuat<float> ---              \n";
    // PureDualQuatf pq0;                   
    // std::cout << "Default Constructor       - PureDualQuatf pq0;                                    : " << pq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 6 scalar values
    PureDualQuatf pq10(1);                                  
    PureDualQuatf pq11(1,2);                                
    PureDualQuatf pq12(1,2,3);               
    PureDualQuatf pq13(1,2,3,4);                                  
    PureDualQuatf pq14(1,2,3,4,5);                                
    PureDualQuatf pq15(1,2,3,4,5,6);        
    std::cout << "Scalar Constructor\n";                              
    std::cout << " - PureDualQuatf pq10{1};                                : " << "\n" << pq10 << "\n";
    std::cout << " - PureDualQuatf pq11{1,2};                              : " << "\n" << pq11 << "\n";
    std::cout << " - PureDualQuatf pq12{1,2,3};                            : " << "\n" << pq12 << "\n";
    std::cout << " - PureDualQuatf pq13{1,2,3,4};                          : " << "\n" << pq13 << "\n";
    std::cout << " - PureDualQuatf pq14{1,2,3,4,5};                        : " << "\n" << pq14 << "\n";
    std::cout << " - PureDualQuatf pq15{1,2,3,4,5,6};                      : " << "\n" << pq15 << "\n";
    // The Copy Constructor accepts another Dual Quaternion of ANY scalar type inside
    PureDualQuatf pq20(pq15);                              
    PureDualQuatf pq21(DualQuat<double>(2,2,2,2,2,2,2,2));             
    std::cout << "Copy Constructor\n";                  
    std::cout << " - PureDualQuatf pq20(pq15);                            : " << "\n" << pq20 << "\n";
    std::cout << " - PureDualQuatf pq21(PureDualQuat<double>(2,2,2,2,2,2,2,2));     : " << "\n" << pq21 << "\n";
    // The Move Constructor ONLY accepts another r-value Pure Dual Quaternion of SAME scalar type inside
    PureDualQuatf pq30(PureDualQuat<float>(3,3,3,3,3,3));          
    std::cout << "Move Constructor\n";                   
    std::cout << " - PureDualQuatf pq30(PureDualQuat<float>(3,3,3,3,3,3));            : " << "\n" << pq30 << "\n";
    // The Unsafe DualQuat Constructor accepts a Dual Quaternion of ANY scalar type inside, the real will set to 0
    PureDualQuatf pq40(q17);                               
    PureDualQuatf pq41(DualQuat<double>(4,4,4,4,4,4,4,4));    
    std::cout << "DualQuat Constructor\n";                  
    std::cout << " - PureDualQuatf pq40(q17);                         : " << "\n" << pq40 << "\n";
    std::cout << " - PureDualQuatf pq41(DualQuat<double>(4,4,4,4,4,4,4,4));   : " << "\n" << pq41 << "\n";


    std::cout << "\nConstructors of float Unit Dual Quaternion --- UnitDualQuat<float> ---              \n";
    // UnitDualQuatf uq0;                   
    // std::cout << "Default Constructor       - UnitDualQuatf uq0;                                    : " << uq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 8 scalar values, normalization is performed for each Constructor
    // ensuring it's a Unit Quaternion
    UnitDualQuatf uq10(1);                                    
    UnitDualQuatf uq11(1,1);                                    
    UnitDualQuatf uq12(1,1,2);                              
    UnitDualQuatf uq13(1,1,2,3);                                
    UnitDualQuatf uq14(1,1,2,3,4);                                       
    UnitDualQuatf uq15(1,1,2,3,4,5);                                     
    UnitDualQuatf uq16(1,1,2,3,4,5,6);                                 
    UnitDualQuatf uq17(1,1,2,3,4,5,6,7);          
    std::cout << "Scalar Constructor\n";                            
    std::cout << " - UnitDualQuatf uq10{1};                                  : " << "\n" << uq10 << "\n";
    std::cout << " - UnitDualQuatf uq11{1,1};                                : " << "\n" << uq11 << "\n";
    std::cout << " - UnitDualQuatf uq12{1,1,2};                              : " << "\n" << uq12 << "\n";
    std::cout << " - UnitDualQuatf uq13{1,1,2,3};                            : " << "\n" << uq13 << "\n";
    std::cout << " - UnitDualQuatf uq14{1,1,2,3,4};                          : " << "\n" << uq14 << "\n";
    std::cout << " - UnitDualQuatf uq15{1,1,2,3,4,5};                        : " << "\n" << uq15 << "\n";
    std::cout << " - UnitDualQuatf uq16{1,1,2,3,4,5,6};                      : " << "\n" << uq16 << "\n";
    std::cout << " - UnitDualQuatf uq17{1,1,2,3,4,5,6,7};                    : " << "\n" << uq17 << "\n";
    // The Copy Constructor accepts another Dual Quaternion of ANY scalar type inside
    UnitDualQuatf uq20(uq17);                            
    UnitDualQuatf uq21(DualQuat<double>(2,2,2,2,2,2,2,2));              
    std::cout << "Copy Constructor\n";                  
    std::cout << " - UnitDualQuatf uq20(uq17);                              : " << "\n" << uq20 << "\n";
    std::cout << " - UnitDualQuatf uq21(DualQuat<double>(2,2,2,2,2,2,2,2));       : " << "\n" << uq21 << "\n";
    // The Move Constructor ONLY accepts another r-value Dual Quaternion of SAME scalar type inside
    UnitDualQuatf uq30(UnitDualQuat<float>(3,3,3,3,3,3,3,3));          
    std::cout << "Move Constructor\n";                
    std::cout << " - UnitDualQuatf uq30(UnitDualQuat<float>(3,3,3,3,3,3,3,3));            : " << "\n" << uq30 << "\n";
    // The Unsafe DualQuat Constructor accepts a Dual Quaternion of ANY scalar type inside, Normalization performed to make sure it's unit
    UnitDualQuatf uq40(q13);                                
    UnitDualQuatf uq41(DualQuat<double>(4,4,4,4,4,4,4,4));       
    std::cout << "DualQuat Constructor\n";               
    std::cout << " - UnitDualQuatf uq40(q13);                           : " << "\n" << uq40 << "\n";
    std::cout << " - UnitDualQuatf uq41(DualQuat<double>(4,4,4,4,4,4,4,4));     : " << "\n" << uq41 << "\n";
    // Construct a Unit Dual Quaternion with a 0 norm is INVALID
    // std::cout << "  - UnitDualQuatf(DualQuat<double>(0));               : " << UnitDualQuatf(DualQuat<double>(0,0,0,0)) << "\n"; // invalid

    std::cout << "\nConstructors of float Unit Pure Dual Quaternion --- UnitPureQuat<float> ---              \n";
    // UnitPureDualQuatf upq0;                   
    // std::cout << "Default Constructor       - UnitPureDualQuatf upq0;                                    : " << upq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 6 scalar values, normalization is performed for each Constructor
    // ensuring it's a Unit Quaternion
    UnitPureDualQuatf upq10(1);                                  
    UnitPureDualQuatf upq11(1,2);                                
    UnitPureDualQuatf upq12(1,2,3);               
    UnitPureDualQuatf upq13(1,2,3,4);                                  
    UnitPureDualQuatf upq14(1,2,3,4,5);                                
    UnitPureDualQuatf upq15(1,2,3,4,5,6);                
    std::cout << "Scalar Constructor\n";
    std::cout << " - UnitPureDualQuatf upq10{1};                                  : " << "\n" << upq10 << "\n";
    std::cout << " - UnitPureDualQuatf upq11{1,2};                                : " << "\n" << upq11 << "\n";
    std::cout << " - UnitPureDualQuatf upq12{1,2,3};                              : " << "\n" << upq12 << "\n";
    std::cout << " - UnitPureDualQuatf upq13{1,2,3,4};                            : " << "\n" << upq13 << "\n";
    std::cout << " - UnitPureDualQuatf upq14{1,2,3,4,5};                          : " << "\n" << upq14 << "\n";
    std::cout << " - UnitPureDualQuatf upq15{1,2,3,4,5,6};                        : " << "\n" << upq15 << "\n";
    // The Copy Constructor accepts another Dual Quaternion of ANY scalar type inside
    UnitPureDualQuatf upq20(upq15);                         
    UnitPureDualQuatf upq21(DualQuat<double>(2,2,2,2,2,2,2,2));            
    std::cout << "Copy Constructor\n";           
    std::cout << " - UnitPureDualQuatf upq20(upq15);                     : " << "\n" << upq20 << "\n";
    std::cout << " - UnitPureDualQuatf upq21(DualQuat<double>(2,2,2,2,2,2,2,2));       : " << "\n" << upq21 << "\n";
    // The Move Constructor ONLY accepts another r-value Dual Quaternion of SAME scalar type inside
    UnitPureDualQuatf upq30(UnitPureDualQuat<float>(3,3,3,3,3,3));          
    std::cout << "Move Constructor\n";                    
    std::cout << " - UnitPureDualQuatf upq30(UnitPureDualQuat<float>(3,3,3,3,3,3));          : " << "\n" << upq30 << "\n";
    // The Unsafe DualQuat Constructor accepts a Dual Quaternion of ANY scalar type inside, Normalization performed to make sure it's unit
    UnitPureDualQuatf upq40(q17);                            
    UnitPureDualQuatf upq41(DualQuat<double>(4,4,4,4,4,4,4,4));       
    std::cout << "DualQuat Constructor\n";           
    std::cout << " - UnitPureDualQuatf upq40(q17);                           : " << "\n" << upq40 << "\n";
    std::cout << " - UnitPureDualQuatf upq41(DualQuat<double>(4,4,4,4,4,4,4,4));     : " << "\n" << upq41 << "\n";
    // Construct a Unit Pure Dual Quaternion form a 0 Dual Quaternion is INVALID
    // std::cout << "  - UnitPureDualQuatf(DualQuat<double>(0,0,0,0));           : " << UnitPureDualQuatf(DualQuat<double>(0,0,0,0)) << "\n"; // invalid

}

void assignments_demo() {

}

int main() {
    constructors_demo();
    assignments_demo();
}

