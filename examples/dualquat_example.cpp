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
    // The default constructor, all values are uninitialized
    // Directly use a default Dual Quaternion without initialization will produce undefined behavior
    // Initialize it before use !!!
    // DualQuatf q0;                   
    // std::cout << "Default constructor       - DualQuatf q0;                   : " << q0 << "\n";  // Undefined behavior
    // A Constructor accepts 0 to 8 scalar values
    DualQuatf q10(1);                                    
    DualQuatf q11(1,1);                                    
    DualQuatf q12(1,1,2);                              
    DualQuatf q13(1,1,2,3);                                
    DualQuatf q14(1,1,2,3,4);                                       
    DualQuatf q15(1,1,2,3,4,5);                                     
    DualQuatf q16(1,1,2,3,4,5,6);                                 
    DualQuatf q17(1,1,2,3,4,5,6,7);                                 
    std::cout << "Scalar constructor       - DualQuatf q10{1};                        : " << q10 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q11{1,1};                      : " << q11 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q12{1,1,2};                    : " << q12 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q13{1,1,2,3};                  : " << q13 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q14{1,1,2,3,4};                : " << q14 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q15{1,1,2,3,4,5};              : " << q15 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q16{1,1,2,3,4,5,6};            : " << q16 << "\n";
    std::cout << "Scalar constructor       - DualQuatf q17{1,1,2,3,4,5,6,7};          : " << q17 << "\n";
    // The Copy constructor accepts another Dual Quaternion of ANY scalar type inside
    DualQuatf q20(q13);                                     
    DualQuatf q21(DualQuat<double>(21,21,21,21,21,21,21,21));                       
    std::cout << "Copy constructor          - DualQuatf q20(q13);                     : " << q20 << "\n";
    std::cout << "Copy constructor          - DualQuatf q21(DualQuat<double>(2));     : " << q21 << "\n";
    // The Move constructor ONLY accepts another r-value Dual Quaternion of SAME scalar type inside
    DualQuatf q30(DualQuat<float>(30,30,30,30,30,30,30,30));                               
    std::cout << "Move constructor          - DualQuatf q30(DualQuatf(3));            : " << q30 << "\n";


    std::cout << "\nConstructors of float Pure Dual Quaternion --- PureDualQuat<float> ---              \n";
    // PureDualQuatf pq0;                   
    // std::cout << "Default constructor       - PureDualQuatf pq0;                                    : " << pq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 6 scalar values
    PureDualQuatf pq10(1);                                  
    PureDualQuatf pq11(1,2);                                
    PureDualQuatf pq12(1,2,3);               
    PureDualQuatf pq13(1,2,3,4);                                  
    PureDualQuatf pq14(1,2,3,4,5);                                
    PureDualQuatf pq15(1,2,3,4,5,6);                                 
    std::cout << "Scalar constructor       - PureDualQuatf pq10{1};                                : " << pq10 << "\n";
    std::cout << "Scalar constructor       - PureDualQuatf pq11{1,2};                              : " << pq11 << "\n";
    std::cout << "Scalar constructor       - PureDualQuatf pq12{1,2,3};                            : " << pq12 << "\n";
    std::cout << "Scalar constructor       - PureDualQuatf pq13{1,2,3,4};                          : " << pq13 << "\n";
    std::cout << "Scalar constructor       - PureDualQuatf pq14{1,2,3,4,5};                        : " << pq14 << "\n";
    std::cout << "Scalar constructor       - PureDualQuatf pq15{1,2,3,4,5,6};                      : " << pq15 << "\n";
    // The Copy constructor accepts another Dual Quaternion of ANY scalar type inside
    PureDualQuatf pq20(pq15);                              
    PureDualQuatf pq21(DualQuat<double>(21,21,21,21,21,21,21,21));           
    std::cout << "Copy constructor          - PureDualQuatf pq20(pq10);                            : " << pq20 << "\n";
    std::cout << "Copy constructor          - PureDualQuatf pq21(PureDualQuat<double>(1,2,3));     : " << pq21 << "\n";
    // The Move constructor ONLY accepts another r-value Pure Dual Quaternion of SAME scalar type inside
    PureDualQuatf pq30(PureDualQuat<float>(30,30,30,30,30,30));                  
    std::cout << "Move constructor          - PureDualQuatf pq30(PureDualQuatf(1,2,3));            : " << pq30 << "\n";
    // The Unsafe DualQuat constructor accepts a Dual Quaternion of ANY scalar type inside, the real will set to 0
    PureDualQuatf pq40(q17);                               
    PureDualQuatf pq41(DualQuat<double>(41,41,41,41,41,41,41,41));             
    std::cout << "Unsafe DualQuat constructor   - PureDualQuatf pq40(q10);                         : " << pq40 << "\n";
    std::cout << "Unsafe DualQuat constructor   - PureDualQuatf pq41(DualQuat<double>(2,1,2,3));   : " << pq41 << "\n";


    std::cout << "\nConstructors of float Unit Dual Quaternion --- UnitDualQuat<float> ---              \n";
    // UnitDualQuatf uq0;                   
    // std::cout << "Default constructor       - UnitDualQuatf uq0;                                    : " << uq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 8 scalar values, normalization is performed for each constructor
    // ensuring it's a Unit Quaternion
    UnitDualQuatf uq10(1);                                    
    UnitDualQuatf uq11(1,1);                                    
    UnitDualQuatf uq12(1,1,2);                              
    UnitDualQuatf uq13(1,1,2,3);                                
    UnitDualQuatf uq14(1,1,2,3,4);                                       
    UnitDualQuatf uq15(1,1,2,3,4,5);                                     
    UnitDualQuatf uq16(1,1,2,3,4,5,6);                                 
    UnitDualQuatf uq17(1,1,2,3,4,5,6,7);                                 
    std::cout << "Scalar constructor       - UnitDualQuatf uq10{1};                                  : " << uq10 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq11{1,1};                                : " << uq11 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq12{1,1,2};                              : " << uq12 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq13{1,1,2,3};                            : " << uq13 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq14{1,1,2,3,4};                          : " << uq14 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq15{1,1,2,3,4,5};                        : " << uq15 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq16{1,1,2,3,4,5,6};                      : " << uq16 << "\n";
    std::cout << "Scalar constructor       - UnitDualQuatf uq17{1,1,2,3,4,5,6,7};                    : " << uq17 << "\n";
    // The Copy constructor accepts another Dual Quaternion of ANY scalar type inside
    UnitDualQuatf uq20(uq17);                               // 1.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    UnitDualQuatf uq21(DualQuat<double>(21,21,21,21,21,21,21,21));            // 0.267261236906 + 0.534522473812 î + 0.801783740520 ĵ + 0.000000000000 k̂
    std::cout << "Copy constructor          - UnitDualQuatf uq20(uq10);                              : " << uq20 << "\n";
    std::cout << "Copy constructor          - UnitDualQuatf uq21(PureDualQuat<double>(1,2,3));       : " << uq21 << "\n";
    // The Move constructor ONLY accepts another r-value Dual Quaternion of SAME scalar type inside
    UnitDualQuatf uq30(UnitDualQuat<float>(30,30,30,30,30,30,30,30));                 // 0.182574182749 + 0.365148365498 î + 0.547722578049 ĵ + 0.730296730995 k̂
    std::cout << "Move constructor          - UnitDualQuatf uq30(UnitDualQuatf(1,2,3,4));            : " << uq30 << "\n";
    // The Unsafe DualQuat constructor accepts a Dual Quaternion of ANY scalar type inside, Normalization performed to make sure it's unit
    UnitDualQuatf uq40(q13);                                // 0.000000000000 + 0.267261236906 î + 0.534522473812 ĵ + 0.801783680916 k̂
    UnitDualQuatf uq41(DualQuat<double>(41,41,41,41,41,41,41,41));              // 0.471404552460 + 0.235702276230 î + 0.471404552460 ĵ + 0.707106828690 k̂
    std::cout << "Unsafe DualQuat constructor   - UnitDualQuatf uq40(q13);                           : " << uq40 << "\n";
    std::cout << "Unsafe DualQuat constructor   - UnitDualQuatf uq41(DualQuat<double>(2,1,2,3));     : " << uq41 << "\n";
    // Construct a Unit Dual Quaternion with a 0 norm is INVALID
    // std::cout << "Unsafe DualQuat constructor    - UnitDualQuatf(DualQuat<double>(0));               : " << UnitDualQuatf(DualQuat<double>(0,0,0,0)) << "\n"; // invalid

    std::cout << "\nConstructors of float Unit Pure Dual Quaternion --- UnitPureQuat<float> ---              \n";
    // UnitPureDualQuatf upq0;                   
    // std::cout << "Default constructor       - UnitPureDualQuatf upq0;                                    : " << upq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 6 scalar values, normalization is performed for each constructor
    // ensuring it's a Unit Quaternion
    UnitPureDualQuatf upq10(1);                                  
    UnitPureDualQuatf upq11(1,2);                                
    UnitPureDualQuatf upq12(1,2,3);               
    UnitPureDualQuatf upq13(1,2,3,4);                                  
    UnitPureDualQuatf upq14(1,2,3,4,5);                                
    UnitPureDualQuatf upq15(1,2,3,4,5,6);                                 
    std::cout << "Scalar constructor       - UnitPureDualQuatf upq10{1};                                : " << upq10 << "\n";
    std::cout << "Scalar constructor       - UnitPureDualQuatf upq11{1,2};                              : " << upq11 << "\n";
    std::cout << "Scalar constructor       - UnitPureDualQuatf upq12{1,2,3};                            : " << upq12 << "\n";
    std::cout << "Scalar constructor       - UnitPureDualQuatf upq13{1,2,3,4};                          : " << upq13 << "\n";
    std::cout << "Scalar constructor       - UnitPureDualQuatf upq14{1,2,3,4,5};                        : " << upq14 << "\n";
    std::cout << "Scalar constructor       - UnitPureDualQuatf upq15{1,2,3,4,5,6};                      : " << upq15 << "\n";
    // The Copy constructor accepts another Dual Quaternion of ANY scalar type inside
    UnitPureDualQuatf upq20(upq15);                         
    UnitPureDualQuatf upq21(DualQuat<double>(21,21,21,21,21,21,21,21));   
    std::cout << "Copy constructor          - UnitPureDualQuatf upq20(upq10);                     : " << upq20 << "\n";
    std::cout << "Copy constructor          - UnitPureDualQuatf upq21(PureDualQuat<double>(1,0,0));   : " << upq21 << "\n";
    // The Move constructor ONLY accepts another r-value Dual Quaternion of SAME scalar type inside
    UnitPureDualQuatf upq30(UnitPureDualQuat<float>(30,30,30,30,30,30));           
    std::cout << "Move constructor          - UnitPureDualQuatf upq30(UnitPureDualQuatf(1,0,0));      : " << upq30 << "\n";
    // The Unsafe DualQuat constructor accepts a Dual Quaternion of ANY scalar type inside, Normalization performed to make sure it's unit
    UnitPureDualQuatf upq40(q17);                            
    UnitPureDualQuatf upq41(DualQuat<double>(41,41,41,41,41,41,41,41));         
    std::cout << "Unsafe DualQuat constructor   - UnitPureDualQuatf upq40(q13);                       : " << upq40 << "\n";
    std::cout << "Unsafe DualQuat constructor   - UnitPureDualQuatf upq41(DualQuat<double>(2,1,2,3));     : " << upq41 << "\n";
    // Construct a Unit Pure Dual Quaternion form a 0 Dual Quaternion is INVALID
    // std::cout << "Unsafe DualQuat constructor    - UnitPureDualQuatf(DualQuat<double>(0,0,0,0));           : " << UnitPureDualQuatf(DualQuat<double>(0,0,0,0)) << "\n"; // invalid

}

void assignments_demo() {

}

int main() {
    constructors_demo();
    assignments_demo();
}

