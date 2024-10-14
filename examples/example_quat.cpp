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
 *     \file examples/quat_example.cpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 */

#include "../src/dqpose/quat.hpp"

void constructors_demo() {
    using namespace dqpose;

    std::cout << " Constructors of float Quaternion --- Quat<float> ---              \n";
    // Quaternion of scalar type of float inside
    // The default constructor, all values are uninitialized
    // Directly use a default Quaternion without initialization will produce undefined behavior
    // Initialize it before use
    // Quatf q0;                   
    // std::cout << "Default constructor       - Quatf q0;                   : " << q0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 4 scalar values
    Quatf q10(0);                                       // 0.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    Quatf q11(0,1);                                     // 0.000000000000 + 1.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    Quatf q12(0,1,2);                                   // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 0.000000000000 k̂
    Quatf q13(0,1,2,3);                                 // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    std::cout << "Scalar constructor       - Quatf q10{0};                : " << q10 << "\n";
    std::cout << "Scalar constructor       - Quatf q11{0,1};              : " << q11 << "\n";
    std::cout << "Scalar constructor       - Quatf q12{0,1,2};            : " << q12 << "\n";
    std::cout << "Scalar constructor       - Quatf q13{0,1,2,3};          : " << q13 << "\n";
    // The Copy constructor accepts another Quaternion of ANY scalar type inside
    Quatf q20(q13);                                     // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    Quatf q21(Quat<double>(2));                         // 2.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    std::cout << "Copy constructor          - Quatf q20(q13);             : " << q20 << "\n";
    std::cout << "Copy constructor          - Quatf q21(Quat<double>(2)); : " << q21 << "\n";
    // The Move constructor ONLY accepts another r-value Quaternion of SAME scalar type inside
    Quatf q30(Quatf(3));                                // 3.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    std::cout << "Move constructor          - Quatf q30(Quatf(3));        : " << q30 << "\n";


    std::cout << "\nConstructors of float Pure Quaternion --- PureQuat<float> ---              \n";
    // PureQuatf pq0;                   
    // std::cout << "Default constructor       - PureQuatf pq0;                                    : " << pq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 3 scalar values
    PureQuatf pq10(1);                                  // 0.000000000000 + 1.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    PureQuatf pq11(1,2);                                // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 0.000000000000 k̂
    PureQuatf pq12(1,2,3);                              // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    std::cout << "Scalar constructor       - PureQuatf pq10{1,};                         : " << pq10 << "\n";
    std::cout << "Scalar constructor       - PureQuatf pq11{1,2};                        : " << pq11 << "\n";
    std::cout << "Scalar constructor       - PureQuatf pq12{1,2,3};                      : " << pq12 << "\n";
    // The Copy constructor accepts another Pure Quaternion of ANY scalar type inside
    PureQuatf pq20(pq10);                               // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    PureQuatf pq21(PureQuat<double>(1,2,3));            // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    std::cout << "Copy constructor          - PureQuatf pq20(pq10);                      : " << pq20 << "\n";
    std::cout << "Copy constructor          - PureQuatf pq21(PureQuat<double>(1,2,3));   : " << pq21 << "\n";
    // The Move constructor ONLY accepts another r-value Pure Quaternion of SAME scalar type inside
    PureQuatf pq30(PureQuatf(1,2,3));                   // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    std::cout << "Move constructor          - PureQuatf pq30(PureQuatf(1,2,3));          : " << pq30 << "\n";
    // The Unsafe Quat constructor accepts a Quaternion of ANY scalar type inside, the real will set to 0
    PureQuatf pq40(q10);                                // 0.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    PureQuatf pq41(Quat<double>(2,1,2,3));              // 0.000000000000 + 1.000000000000 î + 2.000000000000 ĵ + 3.000000000000 k̂
    std::cout << "Unsafe Quat constructor   - PureQuatf pq40(q10);                       : " << pq40 << "\n";
    std::cout << "Unsafe Quat constructor   - PureQuatf pq41(Quat<double>(2,1,2,3));     : " << pq41 << "\n";


    std::cout << "\nConstructors of float Unit Quaternion --- UnitQuat<float> ---              \n";
    // UnitQuatf uq0;                   
    // std::cout << "Default constructor       - UnitQuatf uq0;                                    : " << uq0 << "\n";  // Undefined behavior
    // A Constructor accepts 1 to 4 scalar values, normalization is performed for each constructor
    // ensuring it's a Unit Quaternion
    UnitQuatf uq10{1};                                  // 1.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    UnitQuatf uq11{1,2,3,4};                            // 0.182574182749 + 0.365148365498 î + 0.547722578049 ĵ + 0.730296730995 k̂
    std::cout << "Scalar constructor       - UnitQuatf uq10{1};                          : " << uq10 << "\n";
    std::cout << "Scalar constructor       - UnitQuatf uq10{1,2,3,4};                    : " << uq11 << "\n";
    // The Copy constructor accepts another Pure Quaternion of ANY scalar type inside
    UnitQuatf uq20(uq10);                               // 1.000000000000 + 0.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    UnitQuatf uq21(UnitQuat<double>(1,2,3));            // 0.267261236906 + 0.534522473812 î + 0.801783740520 ĵ + 0.000000000000 k̂
    std::cout << "Copy constructor          - UnitQuatf uq20(uq10);                      : " << uq20 << "\n";
    std::cout << "Copy constructor          - UnitQuatf uq21(PureQuat<double>(1,2,3));   : " << uq21 << "\n";
    // The Move constructor ONLY accepts another r-value Quaternion of SAME scalar type inside
    UnitQuatf uq30(UnitQuatf(1,2,3,4));                 // 0.182574182749 + 0.365148365498 î + 0.547722578049 ĵ + 0.730296730995 k̂
    std::cout << "Move constructor          - UnitQuatf uq30(UnitQuatf(1,2,3,4));        : " << uq30 << "\n";
    // The Unsafe Quat constructor accepts a Quaternion of ANY scalar type inside, Normalization performed to make sure it's unit
    UnitQuatf uq40(q13);                                // 0.000000000000 + 0.267261236906 î + 0.534522473812 ĵ + 0.801783680916 k̂
    UnitQuatf uq41(Quat<double>(2,1,2,3));              // 0.471404552460 + 0.235702276230 î + 0.471404552460 ĵ + 0.707106828690 k̂
    std::cout << "Unsafe Quat constructor   - UnitQuatf uq40(q13);                       : " << uq40 << "\n";
    std::cout << "Unsafe Quat constructor   - UnitQuatf uq41(Quat<double>(2,1,2,3));     : " << uq41 << "\n";
    // Construct a Unit Quaternion with a 0 norm is INVALID
    // std::cout << "Unsafe Quat constructor    - UnitQuatf(Quat<double>(0));               : " << UnitQuatf(Quat<double>(0)) << "\n"; // invalid

    std::cout << "\nConstructors of float Unit Pure Quaternion --- UnitPureQuat<float> ---              \n";
    // UnitPureQuatf upq0;                   
    // std::cout << "Default constructor       - UnitPureQuatf upq0;                                    : " << upq0 << "\n";  // Undefined behavior
    // A Constructor accepts 3 scalar values, normalization is performed for each constructor
    // ensuring it's a Unit Quaternion
    UnitPureQuatf upq10(1,0,0);                          // 0.000000000000 + 1.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    UnitPureQuatf upq11(1,2,3);                          // 0.000000000000 + 0.267261266708 î + 0.534522533417 ĵ + 0.801783800125 k̂
    std::cout << "Scalar constructor       - UnitPureQuatf upq10{1,0,0};                      : " << upq10 << "\n";
    std::cout << "Scalar constructor       - UnitPureQuatf upq11{1,2,3};                      : " << upq11 << "\n";
    // The Copy constructor accepts another Pure Quaternion of ANY scalar type inside
    UnitPureQuatf upq20(upq10);                          // 0.000000000000 + 1.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    UnitPureQuatf upq21(UnitPureQuat<double>(1,0,0));    // 0.000000000000 + 1.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    std::cout << "Copy constructor          - UnitPureQuatf upq20(upq10);                     : " << upq20 << "\n";
    std::cout << "Copy constructor          - UnitPureQuatf upq21(PureQuat<double>(1,0,0));   : " << upq21 << "\n";
    // The Move constructor ONLY accepts another r-value Quaternion of SAME scalar type inside
    UnitPureQuatf upq30(UnitPureQuatf(1,0,0));           // 0.000000000000 + 1.000000000000 î + 0.000000000000 ĵ + 0.000000000000 k̂
    std::cout << "Move constructor          - UnitPureQuatf upq30(UnitPureQuatf(1,0,0));      : " << upq30 << "\n";
    // The Unsafe Quat constructor accepts a Quaternion of ANY scalar type inside, Normalization performed to make sure it's unit
    UnitPureQuatf upq40(q13);                            // 0.000000000000 + 0.267261266708 î + 0.534522533417 ĵ + 0.801783800125 k̂
    UnitPureQuatf upq41(Quat<double>(2,1,2,3));          // 0.000000000000 + 0.267261236906 î + 0.534522473812 ĵ + 0.801783740520 k̂
    std::cout << "Unsafe Quat constructor   - UnitPureQuatf upq40(q13);                       : " << upq40 << "\n";
    std::cout << "Unsafe Quat constructor   - UnitPureQuatf upq41(Quat<double>(2,1,2,3));     : " << upq41 << "\n";
    // Construct a Unit Pure Quaternion form a 0 Quaternion is INVALID
    // std::cout << "Unsafe Quat constructor    - UnitPureQuatf(Quat<double>(0,0,0,0));           : " << UnitPureQuatf(Quat<double>(0,0,0,0)) << "\n"; // invalid

}

void assignments_demo() {

}

int main() {
    constructors_demo();
    assignments_demo();
}

